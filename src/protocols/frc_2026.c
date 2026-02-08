/*
 * The Driver Station Library (LibDS)
 * Copyright (c) 2015-2017 Alex Spataru <alex_spataru@outlook>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/* FRC 2026 protocol - inherits from 2020 with bug fixes, protocol accuracy
 * improvements, and TCP channel support (port 1740). */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <socky.h>
#include <pthread.h>

#include "DS_Utils.h"
#include "DS_Config.h"
#include "DS_Client.h"
#include "DS_Events.h"
#include "DS_Protocol.h"
#include "DS_Joysticks.h"
#include "DS_DefaultProtocols.h"

#if defined _WIN32
#   include <windows.h>
#else
#   include <unistd.h>
#   include <errno.h>
#   include <sys/time.h>
#endif

/*
 * Protocol bytes
 */
static const uint8_t cRequestRestartCode = 0x04;
static const uint8_t cRequestReboot = 0x08;
static const uint8_t cRequestNormal = 0x00;
static const uint8_t cTagCommVersion = 0x01;
static const uint8_t cFMSCommVersion = 0x00;
static const uint8_t cTeleoperated = 0x00;
static const uint8_t cTest = 0x01;
static const uint8_t cAutonomous = 0x02;
static const uint8_t cEnabled = 0x04;
static const uint8_t cEmergencyStop = 0x80;
static const uint8_t cFMSConnected = 0x08;
static const uint8_t cDSConnected = 0x10;
static const uint8_t cFMSRadioPing = 0x10;
static const uint8_t cFMSRobotPing = 0x08;
static const uint8_t cFMSRobotComms = 0x20;
static const uint8_t cRTagCANInfo = 0x0e;
static const uint8_t cRTagCPUInfo = 0x05;
static const uint8_t cRTagRAMInfo = 0x06;
static const uint8_t cRTagDiskInfo = 0x04;
static const uint8_t cRequestTime = 0x01;
static const uint8_t cRobotHasCode = 0x20;
static const uint8_t cRobotBrownout = 0x10;
static const uint8_t cTagDate = 0x0f;
static const uint8_t cTagJoystick = 0x0c;
static const uint8_t cTagTimezone = 0x10;
static const uint8_t cTagCountdown = 0x07;
static const uint8_t cRed1 = 0x00;
static const uint8_t cRed2 = 0x01;
static const uint8_t cRed3 = 0x02;
static const uint8_t cBlue1 = 0x03;
static const uint8_t cBlue2 = 0x04;
static const uint8_t cBlue3 = 0x05;

/*
 * TCP message tag IDs
 */
static const uint8_t cTcpTagJoystickDesc = 0x02;
static const uint8_t cTcpTagMatchInfo = 0x07;
static const uint8_t cTcpTagGameData = 0x0e;
static const uint8_t cTcpTagDisableFaults = 0x04;
static const uint8_t cTcpTagRailFaults = 0x05;
static const uint8_t cTcpTagVersionInfo = 0x0a;
static const uint8_t cTcpTagErrorMessage = 0x0b;
static const uint8_t cTcpTagStdoutMessage = 0x0c;

/*
 * Maximum size (in bytes) for disk and RAM
 */
static const int max_disk_bytes = 512000000;
static const int max_ram_bytes = 256000000;

/*
 * Sent robot and FMS packet counters
 */
static unsigned int send_time_data = 0;
static unsigned int sent_fms_packets = 0;
static unsigned int sent_robot_packets = 0;

/*
 * Control code flags
 */
static int request_reboot = 0;
static int restart_code = 0;

/*
 * FMS connection state tracking for robot port switching
 */
static int fms_was_connected = 0;

/*
 * TCP thread state
 */
static pthread_t tcp_thread;
static pthread_mutex_t tcp_mutex = PTHREAD_MUTEX_INITIALIZER;
static int tcp_running = 0;
static int tcp_connected = 0;
static int tcp_sock = -1;
static int tcp_started = 0;
static int tcp_descriptors_sent = 0;

/* Forward declarations for TCP */
static void tcp_start(void);
static void tcp_stop(void);

/**
 * Obtains the voltage float from the given \a upper and \a lower bytes.
 * FIX: uses /256.0 instead of /255 per protocol spec.
 */
static float decode_voltage(uint8_t upper, uint8_t lower)
{
   return ((float)upper) + ((float)lower / 256.0f);
}

/**
 * Encodes the \a voltage float in the given \a upper and \a lower bytes
 */
static void encode_voltage(float voltage, uint8_t *upper, uint8_t *lower)
{
   if (upper && lower)
   {
      *upper = (uint8_t)(voltage);
      *lower = (uint8_t)((voltage - (int)voltage) * 256);
   }
}

/**
 * Encodes a joystick axis value from [-1.0, 1.0] to a signed byte [-128, 127]
 * cast to uint8_t for wire transmission.
 * FIX: protocol uses signed int8, not unsigned 0-255.
 */
static uint8_t encode_axis(float value)
{
   /* Clamp to [-1.0, 1.0] */
   if (value > 1.0f) value = 1.0f;
   if (value < -1.0f) value = -1.0f;

   /* Map [-1.0, 1.0] to [-128, 127] */
   int scaled;
   if (value < 0)
      scaled = (int)(value * 128.0f);
   else
      scaled = (int)(value * 127.0f);

   if (scaled < -128) scaled = -128;
   if (scaled > 127) scaled = 127;

   return (uint8_t)(int8_t)scaled;
}

/**
 * Extracts 4-byte floats from DS_String objects.
 */
static float extract_float(const DS_String *data, const int start)
{
   char c[4];
   int i;
   for (i = 0; i < 4; i++)
   {
      c[i] = DS_StrCharAt(data, (int)i + start);
   }
   float f;
   memcpy(&f, &c, sizeof(f));
   return f;
}

/**
 * Returns the control code sent to the FMS.
 */
static uint8_t fms_control_code(void)
{
   uint8_t code = 0;

   switch (CFG_GetControlMode())
   {
      case DS_CONTROL_TEST:
         code |= cTest;
         break;
      case DS_CONTROL_AUTONOMOUS:
         code |= cAutonomous;
         break;
      case DS_CONTROL_TELEOPERATED:
         code |= cTeleoperated;
         break;
   }

   if (CFG_GetEmergencyStopped())
      code |= cEmergencyStop;

   if (CFG_GetRobotEnabled())
      code |= cEnabled;

   if (CFG_GetRadioCommunications())
      code |= cFMSRadioPing;

   if (CFG_GetRobotCommunications())
   {
      code |= cFMSRobotComms;
      code |= cFMSRobotPing;
   }

   return code;
}

/**
 * Returns the control code sent to the robot.
 */
static uint8_t get_control_code(void)
{
   uint8_t code = 0;

   switch (CFG_GetControlMode())
   {
      case DS_CONTROL_TEST:
         code |= cTest;
         break;
      case DS_CONTROL_AUTONOMOUS:
         code |= cAutonomous;
         break;
      case DS_CONTROL_TELEOPERATED:
         code |= cTeleoperated;
         break;
      default:
         break;
   }

   if (CFG_GetFMSCommunications())
      code |= cFMSConnected;

   if (CFG_GetEmergencyStopped())
      code |= cEmergencyStop;

   if (CFG_GetRobotEnabled())
      code |= cEnabled;

   return code;
}

/**
 * Generates the request code sent to the robot.
 * FIX: ORs in cDSConnected (0x10) when robot comms are established,
 * signaling to the robot that a DS is connected.
 */
static uint8_t get_request_code(void)
{
   uint8_t code = cRequestNormal;

   if (CFG_GetRobotCommunications())
   {
      /* Always set DS_CONNECTED indicator when we have comms */
      code |= cDSConnected;

      if (request_reboot)
         code = cRequestReboot;
      else if (restart_code)
         code = cRequestRestartCode;
   }

   return code;
}

/**
 * Returns the team station code sent to the robot.
 */
static uint8_t get_station_code(void)
{
   if (CFG_GetPosition() == DS_POSITION_1)
   {
      if (CFG_GetAlliance() == DS_ALLIANCE_RED)
         return cRed1;
      else
         return cBlue1;
   }

   if (CFG_GetPosition() == DS_POSITION_2)
   {
      if (CFG_GetAlliance() == DS_ALLIANCE_RED)
         return cRed2;
      else
         return cBlue2;
   }

   if (CFG_GetPosition() == DS_POSITION_3)
   {
      if (CFG_GetAlliance() == DS_ALLIANCE_RED)
         return cRed3;
      else
         return cBlue3;
   }

   return cRed1;
}

/**
 * Returns the size of the given \a joystick in the UDP packet.
 * FIX: Button byte count is now dynamic: ceil(button_count/8).
 */
static uint8_t get_joystick_size(const int joystick)
{
   int header_size = 2;
   int num_buttons = DS_GetJoystickNumButtons(joystick);
   int button_bytes = (num_buttons + 7) / 8;
   int button_data = button_bytes + 1; /* +1 for the button count byte */
   int axis_data = DS_GetJoystickNumAxes(joystick) + 1;
   int hat_data = (DS_GetJoystickNumHats(joystick) * 2) + 1;

   return header_size + button_data + axis_data + hat_data;
}

/**
 * Returns timezone/date data.
 * FIX: Actually calls time(&rt) to get the current time instead of epoch.
 */
static DS_String get_timezone_data(void)
{
   DS_String data = DS_StrNewLen(13);

   /* Get current time */
   time_t rt;
   time(&rt); /* FIX: was missing, always sent epoch */
   uint32_t ms = 0;
   struct tm timeinfo;

#if defined _WIN32
   localtime_s(&timeinfo, &rt);
#else
   localtime_r(&rt, &timeinfo);
#endif

#if defined _WIN32
   TIME_ZONE_INFORMATION info;
   GetTimeZoneInformation(&info);

   size_t len = wcslen(info.StandardName) + 1;
   char *str = calloc(len, sizeof(char));
   wcstombs_s(NULL, str, len, info.StandardName, wcslen(info.StandardName));
   DS_String tz = DS_StrNew(str);
   free(str);

   GetSystemTime(&info.StandardDate);
   ms = (uint32_t)info.StandardDate.wMilliseconds;
#else
   DS_String tz = DS_StrNew(timeinfo.tm_zone);

   /* Get milliseconds from gettimeofday */
   struct timeval tv;
   gettimeofday(&tv, NULL);
   ms = (uint32_t)(tv.tv_usec / 1000);
#endif

   /* Encode date/time in datagram */
   DS_StrSetChar(&data, 0, (uint8_t)cTagDate);
   DS_StrSetChar(&data, 1, (uint8_t)(ms >> 24));
   DS_StrSetChar(&data, 2, (uint8_t)(ms >> 16));
   DS_StrSetChar(&data, 3, (uint8_t)(ms >> 8));
   DS_StrSetChar(&data, 4, (uint8_t)(ms));
   DS_StrSetChar(&data, 5, (uint8_t)timeinfo.tm_sec);
   DS_StrSetChar(&data, 6, (uint8_t)timeinfo.tm_min);
   DS_StrSetChar(&data, 7, (uint8_t)timeinfo.tm_hour);
   DS_StrSetChar(&data, 8, (uint8_t)timeinfo.tm_yday);
   DS_StrSetChar(&data, 9, (uint8_t)timeinfo.tm_mon);
   DS_StrSetChar(&data, 10, (uint8_t)timeinfo.tm_year);

   /* Add timezone length and tag */
   DS_StrSetChar(&data, 11, cTagTimezone);
   DS_StrSetChar(&data, 12, DS_StrLen(&tz));

   /* Add timezone string */
   DS_StrJoin(&data, &tz);

   return data;
}

/**
 * Constructs joystick data for the UDP packet.
 * FIX: Axis encoding uses signed int8 mapping.
 * FIX: Button encoding uses dynamic byte count with LSB-0 ordering.
 */
static DS_String get_joystick_data(void)
{
   int i = 0;
   int j = 0;
   DS_String data = DS_StrNewLen(0);

   for (i = 0; i < DS_GetJoystickCount(); ++i)
   {
      DS_StrAppend(&data, get_joystick_size(i));
      DS_StrAppend(&data, cTagJoystick);

      /* Add axis data - FIX: use signed int8 encoding */
      DS_StrAppend(&data, DS_GetJoystickNumAxes(i));
      for (j = 0; j < DS_GetJoystickNumAxes(i); ++j)
         DS_StrAppend(&data, encode_axis(DS_GetJoystickAxis(i, j)));

      /* Generate button data - FIX: dynamic byte count, LSB-0 ordering */
      int num_buttons = DS_GetJoystickNumButtons(i);
      int button_bytes = (num_buttons + 7) / 8;

      DS_StrAppend(&data, num_buttons);

      /* Encode buttons into variable-length byte array, LSB-0 */
      for (j = 0; j < button_bytes; ++j)
      {
         uint8_t byte = 0;
         int k;
         for (k = 0; k < 8; ++k)
         {
            int btn_idx = j * 8 + k;
            if (btn_idx < num_buttons && DS_GetJoystickButton(i, btn_idx))
               byte |= (1 << k);
         }
         DS_StrAppend(&data, byte);
      }

      /* Add hat/POV data */
      DS_StrAppend(&data, DS_GetJoystickNumHats(i));
      for (j = 0; j < DS_GetJoystickNumHats(i); ++j)
      {
         DS_StrAppend(&data, (uint8_t)(DS_GetJoystickHat(i, j) >> 8));
         DS_StrAppend(&data, (uint8_t)(DS_GetJoystickHat(i, j)));
      }
   }

   return data;
}

/**
 * Obtains the CPU, RAM, Disk and CAN information from the robot packet
 */
static void read_extended(const DS_String *data, const int offset)
{
   if (!data)
      return;

   uint8_t tag = (uint8_t)DS_StrCharAt(data, offset + 1);

   if (tag == cRTagCANInfo)
   {
      CFG_SetCANUtilization(extract_float(data, offset + 2));
   }
   else if (tag == cRTagCPUInfo)
   {
      double cpu_percent = 0;
      int i;
      for (i = 0; i < 2; i++)
      {
         float t_crit = extract_float(data, offset + 6 + (i * 4));
         float t_above = extract_float(data, offset + 10 + (i * 4));
         float t_norm = extract_float(data, offset + 14 + (i * 4));
         float t_low = extract_float(data, offset + 18 + (i * 4));
         cpu_percent
             += (t_crit + (t_above * 0.90) + (t_norm * 0.75) + (t_low * 0.25)) / (t_crit + t_above + t_norm + t_low);
      }
      cpu_percent /= 2;
      CFG_SetRobotCPUUsage(cpu_percent);
   }
   else if (tag == cRTagRAMInfo)
   {
      CFG_SetRobotRAMUsage((max_ram_bytes - (uint32_t)extract_float(data, offset + 6)) / max_ram_bytes * 100);
   }
   else if (tag == cRTagDiskInfo)
   {
      CFG_SetRobotDiskUsage((max_disk_bytes - extract_float(data, offset + 2)) / max_disk_bytes * 100);
   }
}

/**
 * Gets the alliance type from the received byte
 */
static DS_Alliance get_alliance(const uint8_t byte)
{
   if (byte == cBlue1 || byte == cBlue2 || byte == cBlue3)
      return DS_ALLIANCE_BLUE;

   return DS_ALLIANCE_RED;
}

/**
 * Gets the position type from the received byte
 */
static DS_Position get_position(const uint8_t byte)
{
   if (byte == cRed1 || byte == cBlue1)
      return DS_POSITION_1;
   else if (byte == cRed2 || byte == cBlue2)
      return DS_POSITION_2;
   else if (byte == cRed3 || byte == cBlue3)
      return DS_POSITION_3;

   return DS_POSITION_1;
}

/**
 * Generates a packet that the DS will send to the FMS.
 */
static DS_String create_fms_packet(void)
{
   DS_String data = DS_StrNewLen(8);

   uint8_t integer = 0;
   uint8_t decimal = 0;
   encode_voltage(CFG_GetRobotVoltage(), &integer, &decimal);

   DS_StrSetChar(&data, 0, (sent_fms_packets >> 8));
   DS_StrSetChar(&data, 1, (sent_fms_packets));
   DS_StrSetChar(&data, 2, cFMSCommVersion);
   DS_StrSetChar(&data, 3, fms_control_code());
   DS_StrSetChar(&data, 4, (CFG_GetTeamNumber() >> 8));
   DS_StrSetChar(&data, 5, (CFG_GetTeamNumber()));
   DS_StrSetChar(&data, 6, integer);
   DS_StrSetChar(&data, 7, decimal);

   ++sent_fms_packets;

   return data;
}

/**
 * Generates a packet that the DS will send to the robot.
 * Also handles robot port switching when FMS connected and
 * lazy-starts the TCP thread.
 */
static DS_String create_robot_packet(void)
{
   /* Lazy-start TCP thread on first call */
   if (!tcp_started)
   {
      tcp_start();
      tcp_started = 1;
   }

   /* Check for FMS connection state change for robot port switching */
   int fms_now = CFG_GetFMSCommunications();
   if (fms_now != fms_was_connected)
   {
      fms_was_connected = fms_now;
      /* When FMS is connected, robot port changes from 1110 to 1115 */
      if (DS_CurrentProtocol())
      {
         int new_port = fms_now ? 1115 : 1110;
         if (DS_CurrentProtocol()->robot_socket.out_port != new_port)
         {
            DS_CurrentProtocol()->robot_socket.out_port = new_port;
            /* Force socket reconnection with new port */
            char *address = DS_GetAppliedRobotAddress();
            DS_SocketChangeAddress(&DS_CurrentProtocol()->robot_socket, address);
            DS_FREE(address);
         }
      }
   }

   DS_String data = DS_StrNewLen(6);

   /* Add packet index */
   DS_StrSetChar(&data, 0, (sent_robot_packets >> 8));
   DS_StrSetChar(&data, 1, (sent_robot_packets));

   /* Add packet header */
   DS_StrSetChar(&data, 2, cTagCommVersion);

   /* Add control code, request flags and team station */
   DS_StrSetChar(&data, 3, get_control_code());
   DS_StrSetChar(&data, 4, get_request_code());
   DS_StrSetChar(&data, 5, get_station_code());

   /* Add timezone data (if robot wants it) */
   if (send_time_data)
   {
      DS_String tz = get_timezone_data();
      DS_StrJoin(&data, &tz);
   }
   /* Add joystick data */
   else if (sent_robot_packets > 5)
   {
      DS_String js = get_joystick_data();
      DS_StrJoin(&data, &js);
   }

   /* Add countdown tag if match time is being tracked */
   if (CFG_GetMatchTime() > 0)
   {
      float match_time = CFG_GetMatchTime();
      DS_StrAppend(&data, 5); /* tag size: 1 byte tag + 4 byte float */
      DS_StrAppend(&data, cTagCountdown);
      /* Append float as 4 bytes (big-endian) */
      uint8_t fbytes[4];
      memcpy(fbytes, &match_time, 4);
      DS_StrAppend(&data, fbytes[0]);
      DS_StrAppend(&data, fbytes[1]);
      DS_StrAppend(&data, fbytes[2]);
      DS_StrAppend(&data, fbytes[3]);
   }

   ++sent_robot_packets;

   return data;
}

/**
 * Interprets an FMS packet.
 * Now parses match number, tournament level, and remaining time.
 */
static int read_fms_packet(const DS_String *data)
{
   if (!data)
      return 0;

   if (DS_StrLen(data) < 22)
      return 0;

   uint8_t control = (uint8_t)DS_StrCharAt(data, 3);
   uint8_t station = (uint8_t)DS_StrCharAt(data, 5);

   /* Change robot enabled state */
   CFG_SetRobotEnabled(control & cEnabled);

   /* Get FMS robot mode */
   if (control & cTeleoperated)
      CFG_SetControlMode(DS_CONTROL_TELEOPERATED);
   else if (control & cAutonomous)
      CFG_SetControlMode(DS_CONTROL_AUTONOMOUS);
   else if (control & cTest)
      CFG_SetControlMode(DS_CONTROL_TEST);

   /* Get FMS Enable state */
   if (control & cEnabled)
      CFG_SetRobotEnabled(1);

   /* Update alliance and position */
   CFG_SetAlliance(get_alliance(station));
   CFG_SetPosition(get_position(station));

   /* Parse tournament level (byte 6) */
   if (DS_StrLen(data) > 6)
   {
      uint8_t tourn_level = (uint8_t)DS_StrCharAt(data, 6);
      CFG_SetTournamentLevel(tourn_level);
   }

   /* Parse match number (bytes 7-8, big-endian) */
   if (DS_StrLen(data) > 8)
   {
      uint16_t match_num = ((uint8_t)DS_StrCharAt(data, 7) << 8)
                         | (uint8_t)DS_StrCharAt(data, 8);
      CFG_SetMatchNumber(match_num);
   }

   /* Parse remaining match time (bytes 20-21, big-endian) */
   if (DS_StrLen(data) > 21)
   {
      uint16_t remain = ((uint8_t)DS_StrCharAt(data, 20) << 8)
                      | (uint8_t)DS_StrCharAt(data, 21);
      CFG_SetMatchTime((float)remain);
   }

   return 1;
}

/**
 * Interprets a robot packet.
 * Now also parses brownout flag from robot status byte.
 */
static int read_robot_packet(const DS_String *data)
{
   if (!data)
      return 0;

   if (DS_StrLen(data) < 7)
      return 0;

   uint8_t control = (uint8_t)DS_StrCharAt(data, 3);
   uint8_t rstatus = (uint8_t)DS_StrCharAt(data, 4);
   uint8_t request = (uint8_t)DS_StrCharAt(data, 7);

   /* Update client information */
   CFG_SetRobotCode(rstatus & cRobotHasCode);
   CFG_SetEmergencyStopped(control & cEmergencyStop);

   /* Parse brownout flag (bit 4 of status byte) */
   CFG_SetRobotBrownout(rstatus & cRobotBrownout);

   /* Update date/time request flag */
   send_time_data = (request == cRequestTime);

   /* Calculate the voltage */
   uint8_t upper = (uint8_t)DS_StrCharAt(data, 5);
   uint8_t lower = (uint8_t)DS_StrCharAt(data, 6);
   CFG_SetRobotVoltage(decode_voltage(upper, lower));

   /* Read extended data if present */
   if (DS_StrLen(data) > 9)
      read_extended(data, 8);

   return 1;
}

/* ========================================================================= */
/*  TCP Channel Implementation (Port 1740)                                   */
/* ========================================================================= */

/**
 * Send a length-prefixed TCP message: [2-byte size][1-byte tag][payload]
 */
static int tcp_send_message(uint8_t tag, const uint8_t *payload, int payload_len)
{
   pthread_mutex_lock(&tcp_mutex);

   if (!tcp_connected || tcp_sock < 0)
   {
      pthread_mutex_unlock(&tcp_mutex);
      return -1;
   }

   /* Total message size = 1 (tag) + payload_len */
   uint16_t msg_size = 1 + payload_len;
   uint8_t header[3];
   header[0] = (uint8_t)(msg_size >> 8);
   header[1] = (uint8_t)(msg_size & 0xFF);
   header[2] = tag;

   int sent = send(tcp_sock, (const char *)header, 3, 0);
   if (sent <= 0)
   {
      tcp_connected = 0;
      pthread_mutex_unlock(&tcp_mutex);
      return -1;
   }

   if (payload_len > 0 && payload)
   {
      sent = send(tcp_sock, (const char *)payload, payload_len, 0);
      if (sent <= 0)
      {
         tcp_connected = 0;
         pthread_mutex_unlock(&tcp_mutex);
         return -1;
      }
   }

   pthread_mutex_unlock(&tcp_mutex);
   return 0;
}

/**
 * Send joystick descriptors (Tag 0x02) for all registered joysticks.
 * Format per joystick: [name_len][name][axis_count][axis_types...][button_count][POV_count]
 * We send a generic name and axis type 1 (analog) for each axis.
 */
static void tcp_send_joystick_descriptors(void)
{
   int i, j;
   for (i = 0; i < DS_GetJoystickCount(); ++i)
   {
      int num_axes = DS_GetJoystickNumAxes(i);
      int num_buttons = DS_GetJoystickNumButtons(i);
      int num_hats = DS_GetJoystickNumHats(i);

      /* Build descriptor payload */
      char name[32];
      snprintf(name, sizeof(name), "Joystick %d", i);
      int name_len = (int)strlen(name);

      /* Payload: [joystick_index][is_xbox(0)][type(0=unknown)]
       *          [name_len][name...][axis_count][axis_types...]
       *          [button_count][POV_count] */
      int payload_len = 3 + 1 + name_len + 1 + num_axes + 1 + 1;
      uint8_t *payload = (uint8_t *)calloc(payload_len, 1);

      int pos = 0;
      payload[pos++] = (uint8_t)i;        /* joystick index */
      payload[pos++] = 0;                  /* is_xbox */
      payload[pos++] = 0;                  /* type (unknown) */
      payload[pos++] = (uint8_t)name_len;  /* name length */
      memcpy(payload + pos, name, name_len);
      pos += name_len;
      payload[pos++] = (uint8_t)num_axes;
      for (j = 0; j < num_axes; ++j)
         payload[pos++] = 1; /* axis type: analog */
      payload[pos++] = (uint8_t)num_buttons;
      payload[pos++] = (uint8_t)num_hats;

      tcp_send_message(cTcpTagJoystickDesc, payload, pos);
      free(payload);
   }
}

/**
 * Send match info (Tag 0x07).
 * Format: [comp_name_len][comp_name...][match_type]
 */
static void tcp_send_match_info(void)
{
   const char *comp_name = "FRC Competition";
   int name_len = (int)strlen(comp_name);

   int payload_len = 1 + name_len + 1;
   uint8_t *payload = (uint8_t *)calloc(payload_len, 1);

   int pos = 0;
   payload[pos++] = (uint8_t)name_len;
   memcpy(payload + pos, comp_name, name_len);
   pos += name_len;
   payload[pos++] = 0; /* match type: none/practice */

   tcp_send_message(cTcpTagMatchInfo, payload, pos);
   free(payload);
}

/**
 * Send game-specific data (Tag 0x0e).
 */
static void tcp_send_game_data(void)
{
   char *game_data = DS_StrToChar(CFG_GetGameData());
   if (game_data)
   {
      int gd_len = (int)strlen(game_data);
      tcp_send_message(cTcpTagGameData, (const uint8_t *)game_data, gd_len);
      DS_FREE(game_data);
   }
}

/**
 * Process a received TCP message from the robot.
 */
static void tcp_process_message(uint8_t tag, const uint8_t *payload, int payload_len)
{
   if (tag == cTcpTagVersionInfo && payload_len > 0)
   {
      /* Version info: [device_type][name_len][name...][version_len][version...] */
      /* Feed as a netconsole message */
      if (payload_len >= 3)
      {
         int pos = 0;
         /* uint8_t device_type = payload[pos]; */
         pos++;
         int name_len = payload[pos++];
         if (pos + name_len <= payload_len)
         {
            char name_buf[256];
            int copy_len = name_len < 255 ? name_len : 255;
            memcpy(name_buf, payload + pos, copy_len);
            name_buf[copy_len] = '\0';
            pos += name_len;

            if (pos < payload_len)
            {
               int ver_len = payload[pos++];
               char ver_buf[256];
               copy_len = ver_len < 255 ? ver_len : 255;
               if (pos + copy_len <= payload_len)
               {
                  memcpy(ver_buf, payload + pos, copy_len);
                  ver_buf[copy_len] = '\0';

                  DS_String msg = DS_StrFormat("Robot: %s v%s", name_buf, ver_buf);
                  CFG_AddNetConsoleMessage(&msg);
                  DS_StrRmBuf(&msg);
               }
            }
         }
      }
   }
   else if (tag == cTcpTagErrorMessage && payload_len > 0)
   {
      /* Error messages: feed into netconsole */
      /* Format: [timestamp(4)][seq(2)][error_code(4)][flags(1)]
       *         [details_len(2)][details...][location_len(2)][location...]
       *         [callstack_len(2)][callstack...] */
      int pos = 0;

      /* Skip timestamp (4 bytes) and sequence (2 bytes) */
      pos += 6;
      if (pos + 4 > payload_len) return;

      /* Skip error code (4 bytes) and flags (1 byte) */
      pos += 5;
      if (pos + 2 > payload_len) return;

      /* Read details */
      uint16_t details_len = (payload[pos] << 8) | payload[pos + 1];
      pos += 2;
      char details_buf[1024];
      int copy_len = details_len < 1023 ? details_len : 1023;
      if (pos + copy_len <= payload_len)
      {
         memcpy(details_buf, payload + pos, copy_len);
         details_buf[copy_len] = '\0';
         pos += details_len;

         /* Read location */
         char location_buf[512] = "";
         if (pos + 2 <= payload_len)
         {
            uint16_t loc_len = (payload[pos] << 8) | payload[pos + 1];
            pos += 2;
            int loc_copy = loc_len < 511 ? loc_len : 511;
            if (pos + loc_copy <= payload_len)
            {
               memcpy(location_buf, payload + pos, loc_copy);
               location_buf[loc_copy] = '\0';
            }
         }

         /* Format and send as netconsole message */
         DS_String msg;
         if (strlen(location_buf) > 0)
            msg = DS_StrFormat("<font color=#f44>ERROR at %s: %s</font>", location_buf, details_buf);
         else
            msg = DS_StrFormat("<font color=#f44>ERROR: %s</font>", details_buf);
         CFG_AddNetConsoleMessage(&msg);
         DS_StrRmBuf(&msg);
      }
   }
   else if (tag == cTcpTagStdoutMessage && payload_len > 0)
   {
      /* Stdout messages: [timestamp(4)][seq(2)][text...] */
      if (payload_len > 6)
      {
         int text_len = payload_len - 6;
         char *text_buf = (char *)calloc(text_len + 1, 1);
         memcpy(text_buf, payload + 6, text_len);
         text_buf[text_len] = '\0';

         DS_String msg = DS_StrNew(text_buf);
         CFG_AddNetConsoleMessage(&msg);
         DS_StrRmBuf(&msg);
         free(text_buf);
      }
   }
   else if (tag == cTcpTagDisableFaults && payload_len > 0)
   {
      DS_String msg = DS_StrNew("<font color=#fa0>Robot reported disable faults</font>");
      CFG_AddNetConsoleMessage(&msg);
      DS_StrRmBuf(&msg);
   }
   else if (tag == cTcpTagRailFaults && payload_len > 0)
   {
      DS_String msg = DS_StrNew("<font color=#fa0>Robot reported rail faults</font>");
      CFG_AddNetConsoleMessage(&msg);
      DS_StrRmBuf(&msg);
   }
}

/**
 * TCP receive loop: reads length-prefixed messages from the robot.
 */
static void tcp_recv_loop(void)
{
   uint8_t recv_buf[8192];

   while (tcp_running)
   {
      pthread_mutex_lock(&tcp_mutex);
      int sock = tcp_sock;
      int connected = tcp_connected;
      pthread_mutex_unlock(&tcp_mutex);

      if (!connected || sock < 0)
         break;

      /* Read 2-byte length header */
      uint8_t len_buf[2];
      int r = recv(sock, (char *)len_buf, 2, MSG_WAITALL);
      if (r <= 0)
         break;

      uint16_t msg_len = (len_buf[0] << 8) | len_buf[1];
      if (msg_len == 0 || msg_len > sizeof(recv_buf))
         continue;

      /* Read the message body */
      r = recv(sock, (char *)recv_buf, msg_len, MSG_WAITALL);
      if (r <= 0)
         break;

      /* First byte is the tag */
      uint8_t tag = recv_buf[0];
      tcp_process_message(tag, recv_buf + 1, r - 1);
   }

   pthread_mutex_lock(&tcp_mutex);
   tcp_connected = 0;
   pthread_mutex_unlock(&tcp_mutex);
}

/**
 * TCP thread function: connect/reconnect loop on port 1740.
 */
static void *tcp_thread_func(void *arg)
{
   (void)arg;

   while (tcp_running)
   {
      /* Wait for robot communications before attempting TCP */
      if (!CFG_GetRobotCommunications())
      {
#if defined _WIN32
         Sleep(500);
#else
         usleep(500000);
#endif
         continue;
      }

      /* Get the robot address */
      char *robot_addr = DS_GetAppliedRobotAddress();
      if (!robot_addr || strlen(robot_addr) == 0)
      {
         DS_FREE(robot_addr);
#if defined _WIN32
         Sleep(1000);
#else
         usleep(1000000);
#endif
         continue;
      }

      /* Attempt TCP connection to port 1740 */
      int sock = create_client_tcp(robot_addr, "1740", SOCKY_IPv4, 0);
      DS_FREE(robot_addr);

      if (sock > 0)
      {
         pthread_mutex_lock(&tcp_mutex);
         tcp_sock = sock;
         tcp_connected = 1;
         tcp_descriptors_sent = 0;
         pthread_mutex_unlock(&tcp_mutex);

         /* Send initial data */
         tcp_send_joystick_descriptors();
         tcp_send_match_info();
         tcp_send_game_data();

         pthread_mutex_lock(&tcp_mutex);
         tcp_descriptors_sent = 1;
         pthread_mutex_unlock(&tcp_mutex);

         /* Enter receive loop */
         tcp_recv_loop();

         /* Connection lost, clean up */
         pthread_mutex_lock(&tcp_mutex);
         if (tcp_sock > 0)
         {
            socket_close(tcp_sock);
            tcp_sock = -1;
         }
         tcp_connected = 0;
         pthread_mutex_unlock(&tcp_mutex);
      }

      if (!tcp_running)
         break;

      /* Wait before reconnecting */
#if defined _WIN32
      Sleep(2000);
#else
      usleep(2000000);
#endif
   }

   return NULL;
}

/**
 * Start the TCP thread.
 */
static void tcp_start(void)
{
   pthread_mutex_lock(&tcp_mutex);
   if (tcp_running)
   {
      pthread_mutex_unlock(&tcp_mutex);
      return;
   }
   tcp_running = 1;
   pthread_mutex_unlock(&tcp_mutex);

   pthread_create(&tcp_thread, NULL, tcp_thread_func, NULL);
}

/**
 * Stop the TCP thread and close the connection.
 */
static void tcp_stop(void)
{
   pthread_mutex_lock(&tcp_mutex);
   tcp_running = 0;
   if (tcp_sock > 0)
   {
      socket_close(tcp_sock);
      tcp_sock = -1;
   }
   tcp_connected = 0;
   pthread_mutex_unlock(&tcp_mutex);

   /* Wait for thread to finish (with timeout) */
   if (tcp_started)
   {
      pthread_join(tcp_thread, NULL);
      tcp_started = 0;
   }
}

/**
 * Called when the robot watchdog expires, resets the control code flags
 * and tears down the TCP connection (will auto-reconnect).
 */
static void reset_robot(void)
{
   request_reboot = 0;
   restart_code = 0;
   send_time_data = 0;
   fms_was_connected = 0;
   tcp_descriptors_sent = 0;

   /* Disconnect TCP (will auto-reconnect when robot comes back) */
   pthread_mutex_lock(&tcp_mutex);
   if (tcp_sock > 0)
   {
      socket_close(tcp_sock);
      tcp_sock = -1;
   }
   tcp_connected = 0;
   pthread_mutex_unlock(&tcp_mutex);
}

/**
 * Updates the control code flags to instruct the roboRIO to reboot
 */
static void reboot_robot(void)
{
   request_reboot = 1;
}

/**
 * Updates the control code flags to instruct the robot to restart code
 */
static void restart_robot_code(void)
{
   restart_code = 1;
}

/**
 * Initializes and configures the FRC 2026 communication protocol.
 * Inherits from FRC 2020 and overrides with bug-fixed implementations.
 */
DS_Protocol DS_GetProtocolFRC_2026(void)
{
   DS_Protocol protocol = DS_GetProtocolFRC_2020();

   /* Set packet generator functions */
   protocol.create_fms_packet = &create_fms_packet;
   protocol.create_robot_packet = &create_robot_packet;

   /* Set packet interpretation functions */
   protocol.read_fms_packet = &read_fms_packet;
   protocol.read_robot_packet = &read_robot_packet;

   /* Set reset functions */
   protocol.reset_robot = &reset_robot;
   protocol.reboot_robot = &reboot_robot;
   protocol.restart_robot_code = &restart_robot_code;

   /* FMS receive port update: official FMS uses 1121 */
   protocol.fms_socket.in_port = 1121;

   /* Set protocol name */
   DS_StrRmBuf(&protocol.name);
   protocol.name = DS_StrNew("FRC 2026");

   /* tcp_stop() is available for clean shutdown when needed */
   (void)tcp_stop;

   return protocol;
}
