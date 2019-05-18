//
// OTA host example for BLED112. Version 0.2 / May 17, 2018
//
//
// This is free software distributed under the terms of the MIT license reproduced below.
//
// Copyright 2017 Silicon Labs
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "debug.h"

#include <sys/time.h>

#include "cmd_def.h"
#include "uart.h"
#include "ebl_utils.h"

#define CLARG_PORT 1
#define CLARG_ACTION 2
#define CLARG_OPTIONS 3

#define UART_TIMEOUT 1000

#define MAX_DEVICES 64
int found_devices_count = 0;
bd_addr found_devices[MAX_DEVICES];

enum actions {
	action_none,
	action_scan,
	action_connect,
	action_info,
};
enum actions action = action_none;

typedef enum {
	state_disconnected,
	state_connecting,
	state_connected,
	state_finding_services,
	state_finding_attributes,
	state_read_version,
	state_starting_ota,
	state_reset_to_dfu,
	state_ebl_upload,
	state_ota_finish,
	state_ota_disconnect,
	state_finish,
	state_last
} states;
states state = state_disconnected;

const char *state_names[state_last] = {
		"disconnected",
		"connecting",
		"connected",
		"finding_services",
		"finding_attributes",
		"starting_ota",
		"reset to DFU",
		"ebl_upload",
		"ota_finish",
		"ota_disconnect"
		"finish"
};


#define FIRST_HANDLE 0x0001
#define LAST_HANDLE  0xffff


// Silicon labs OTA service: 1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0
static const uint8 OTA_SERVICE_UUID[16] = {0xf0, 0x19, 0x21, 0xb4, 0x47, 0x8f, 0xa4, 0xbf, 0xa1, 0x4f, 0x63, 0xfd, 0xee, 0xd6, 0x14, 0x1d};

// OTA control characteristic: f7bf3564-fb6d-4e53-88a4-5e37e0326063
static const uint8 OTA_CTRL_UUID[16] = {0x63, 0x60, 0x32, 0xe0, 0x37, 0x5e, 0xa4, 0x88, 0x53, 0x4e, 0x6d, 0xfb, 0x64, 0x35, 0xbf, 0xf7};

// OTA data characteristic: 984227f3-34fc-4045-a5d0-2c581f81a153
static const uint8 OTA_DATA_UUID[16] = {0x53, 0xa1, 0x81, 0x1f, 0x58, 0x2c, 0xd0, 0xa5, 0x45, 0x40, 0xfc, 0x34, 0xf3, 0x27, 0x42, 0x98};

// BLE stack version: 4f4a2368-8cca-451ebfff-cf0e2ee23e9f
static const uint8 STACK_VERSION_UUID[16] = {0x9f, 0x3e, 0xe2, 0x2e, 0x0e, 0xcf, 0xff, 0xbf, 0x1e, 0x45, 0xca, 0x8c, 0x68, 0x23, 0x4a, 0x4f};

// OTA version: 4cc07bcf-0868-4b32-9dad-ba4cc41e5316
static const uint8 OTA_VERSION_UUID[16] =  {0x16, 0x53, 0x1e, 0xc4, 0x4c, 0xba, 0xad, 0x9d, 0x32, 0x4b, 0x68, 0x08, 0xcf, 0x7b, 0xc0, 0x4c};

// Gecko bootloader version: 25f05c0a-e917-46e9-b2a5-aa2be1245afe
static const uint8 BLD_VERSION_UUID[16] =  {0xfe, 0x5a, 0x24, 0xe1, 0x2b, 0xaa, 0xa5, 0xb2, 0xe9, 0x46, 0x17, 0xe9, 0x0a, 0x5c, 0xf0, 0x25};

// Application version: 0d77cc11-4ac1-49f2-bfa9-cd96ac7a92f8
static const uint8 APP_VERSION_UUID[16] =  {0xf8, 0x92, 0x7a, 0xac, 0x96, 0xcd, 0xa9, 0xbf, 0xf2, 0x49, 0xc1, 0x4a, 0x11, 0xcc, 0x77, 0x0d};


/** dfu file to upload*/
FILE *dfu_file=NULL;

///DFU
#define MAX_DFU_PACKET 20
uint8 dfu_data[MAX_DFU_PACKET];

size_t dfu_toload = 0;
size_t dfu_total = 0;
size_t dfu_current_pos = 0;
time_t dfu_start_time;

size_t dfu_size;

uint8 primary_service_uuid[] = {0x00, 0x28};


uint16 ota_handles_start = 0;
uint16 ota_handles_end = 0;

uint16 ota_ctrl_handle = 0;
uint16 ota_data_handle = 0;
uint16 sdk_vers_handle = 0;
uint16 ota_vers_handle = 0;
uint16 bld_vers_handle = 0;
uint16 app_vers_handle = 0;

uint8 ota_version = 0xFF;

bd_addr connect_addr;

static int conn_status = 0;
static int conn_handle = -1;

static uint32 scan_countdown;

void usage(char *exe)
{
	printf("--- Usage: ----\n");
	printf("%s list                        => check COM port number of BLED112\n", exe);
	printf("%s <COMx> scan                 => scan for BLE devices (stops automatically after 5 seconds)\n", exe);
	printf("%s <COMx> <address> <GBL file> => run OTA update\n", exe);

}

void change_state(states new_state)
{
#ifdef DEBUG
	printf("DEBUG: State changed: %s --> %s\n", state_names[state], state_names[new_state]);
#endif
	state = new_state;
}

int dfu_read_size()
{
	if (fseek(dfu_file, 0L, SEEK_END))
		return -1;
	dfu_total = dfu_toload = ftell(dfu_file);
	if (fseek(dfu_file, 0L, SEEK_SET))
		return -1;
	printf("Bytes to send:%d\n", (int)dfu_toload); fflush(stdout);
	return 0;
}

/*
 *  Send one block of EBL data by writing to the ota_data characteristic.
 *  If param 'retry' is 0 then data is read from the EBL file that was passed
 *  as command line argument.
 *  retry > 0 means that we are re-sending data (because previous write command failed)
 * */
void send_ebl_data(int retry)
{
#if 1
	static int num_calls = 0;

	printf("send_ebl_data %d, retry = %d, bytes_left: %d\n", ++num_calls, retry, dfu_toload);
#endif

	if(dfu_toload <= 0)
	{
		printf("all data already sent!??\n");
		return;
	}

	if(retry == 0)
	{
		dfu_size = dfu_toload > sizeof(dfu_data) ? sizeof(dfu_data) : dfu_toload;
		if (fread(dfu_data, 1, dfu_size, dfu_file) != dfu_size)
		{
			printf("File read failure\n");
			exit(-1);
		}
	}

	ble_cmd_attclient_write_command(conn_handle, ota_data_handle, dfu_size, dfu_data);

}

/**
 * Compare Bluetooth addresses
 *
 * @param first First address
 * @param second Second address
 * @return Zero if addresses are equal
 */
int cmp_bdaddr(bd_addr first, bd_addr second)
{
	int i;
	for (i = 0; i < sizeof(bd_addr); i++) {
		if (first.addr[i] != second.addr[i]) return 1;
	}
	return 0;
}

void print_bdaddr(bd_addr bdaddr)
{
    printf("%02x:%02x:%02x:%02x:%02x:%02x",
            bdaddr.addr[5],
            bdaddr.addr[4],
            bdaddr.addr[3],
            bdaddr.addr[2],
            bdaddr.addr[1],
            bdaddr.addr[0]);
}

void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
	printf("Incoming packet: ");
	int i;
	for (i = 0; i < sizeof(*hdr); i++) {
		printf("%02x ", ((unsigned char *)hdr)[i]);
	}
	for (i = 0; i < hdr->lolen; i++) {
		printf("%02x ", data[i]);
	}
	printf("\n");
}

void output(uint8 len1, uint8* data1, uint16 len2, uint8* data2)
{
    if (uart_tx(len1, data1) || uart_tx(len2, data2)) {
        printf("ERROR: Writing to serial port failed\n");
        exit(1);
    }
}

int read_message(int timeout_ms)
{
    unsigned char data[256]; // enough for BLE
    struct ble_header hdr;
    int r;

    r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, UART_TIMEOUT);
    if (!r) {
        return -1; // timeout
    }
    else if (r < 0) {
        printf("ERROR: Reading header failed. Error code:%d\n", r);
        return 1;
    }

    //dump_ble_header(&hdr);
    
    if (hdr.lolen) {
        r = uart_rx(hdr.lolen, data, UART_TIMEOUT);
        if (r <= 0) {
            printf("ERROR: Reading data failed. Error code:%d\n", r);
            return 1;
        }
    }

    const struct ble_msg *msg = ble_get_msg_hdr(hdr);
    //dump_ble_msg(msg,data);

#ifdef DEBUG
    print_raw_packet(&hdr, data);
#endif
    
    if (!msg) {
        printf("ERROR: Unknown message received\n");
        exit(1);
    }

    msg->handler(data);

    return 0;
}


void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
    printf("Build: %u, protocol_version: %u, hardware: ", msg->build, msg->protocol_version);
    switch (msg->hw) {
    case 0x01: printf("BLE112"); break;
    case 0x02: printf("BLED112"); break;
    default: printf("Unknown");
    }
    printf("\n");

    if (action == action_info) change_state(state_finish);
}

/*
 * Decoding of received advertisement packets
 * */
void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{

	int i;
    char *name = NULL;
    int found = 0;

    if (found_devices_count >= MAX_DEVICES) change_state(state_finish);

    // Check if this device already found
    for (i = 0; i < found_devices_count; i++) {
        if (!cmp_bdaddr(msg->sender, found_devices[i]))
        {
        	found = 1;
        	// print each found device only once.
        	return;
        }
    }

    if(found == 0)
    {
    	found_devices_count++;
    	memcpy(found_devices[i].addr, msg->sender.addr, sizeof(bd_addr));
    }

    // Parse data
    for (i = 0; i < msg->data.len; ) {
        int8 len = msg->data.data[i++];
        if (!len) continue;
        if (i + len > msg->data.len) break; // not enough data
        uint8 type = msg->data.data[i++];
        switch (type) {
        case 0x09: // complete local name
        case 0x08: // shortened local name
            name = malloc(len);
            memcpy(name, msg->data.data + i, len - 1);
            name[len - 1] = '\0';
        }

        i += len - 1;
    }

    printf("[%d] ", found_devices_count);
    print_bdaddr(msg->sender);
    printf(" RSSI:%d", msg->rssi);

    printf(" Name:");
    if (name) printf("%s", name);
    else printf("Unknown");
    printf("\n");

    free(name);
}

void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
    // New connection
    if (msg->flags & connection_connected) {

    	if(conn_status == 0)
    	{
    		// first connection status event (new connection established)

    		// store connection handle (this is needed for example when disconnecting)
    		conn_handle = msg->connection;

    		change_state(state_connected);
    		printf("Connected (handle %u)\n", conn_handle);

    		change_state(state_finding_services);
    		ble_cmd_attclient_read_by_group_type(msg->connection, FIRST_HANDLE, LAST_HANDLE, 2, primary_service_uuid);

    		ble_cmd_connection_update(msg->connection, 6,12,0,1000);

    		conn_status = 1;
    	}
    	else
    	{
    		// connection status event received while already connected. This will happen if for example the slave
    		// changes connection parameters
    		printf("Connection updated ->\n");
    	}

    	// connection parameters:
    	printf("Connection interval: %u units = %.2f ms\n", msg->conn_interval, (float)(msg->conn_interval) * 1.25);
    	printf("timeout: %u units = %u ms\n", msg->timeout, (msg->timeout) * 10);

    }

}

void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
#ifdef DEBUG
	int i;
#endif

    if (msg->uuid.len == 0)
    	return;

#ifdef DEBUG
    printf("Service UUID:");
    for(i=0;i<16;i++)
    {
    	printf("%2.2x_", msg->uuid.data[i]);
    }
    printf("\n");
#endif

    // try to find OTA service (uses custom 128bit UUID)
    if (state == state_finding_services && ota_handles_start == 0 && msg->uuid.len == 16) {

    	if(memcmp(msg->uuid.data, OTA_SERVICE_UUID, 16) == 0)
    	{
    		ota_handles_start = msg->start;
    		ota_handles_end = msg->end;
    		printf("Found OTA service, handles %u..%u\n", ota_handles_start, ota_handles_end);
    	}
    }

}

void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
    if (state == state_finding_services) {

        if (ota_handles_start == 0) {
            printf("service not found\n");
            change_state(state_finish);
        }
        else {
        	// OTA service found -> next step is to discover the characteristics
            change_state(state_finding_attributes);
            ble_cmd_attclient_find_information(msg->connection, ota_handles_start, ota_handles_end);
        }
    }
    else if (state == state_finding_attributes) {
        // Client characteristic configuration not found
        if (ota_ctrl_handle == 0) {
            printf("OTA control characteristic not found!\n");
            change_state(state_finish);
        }
        // start OTA by writing value 0x00 to ota_control characteristic
        else {
        	const uint8 OTA_START = 0x00;


        	if(ota_data_handle == 0)
        	{
        		// no OTA data characteristic -> target device is not in OTA mode.
        		printf("Target not in OTA mode, requesting DFU reboot...\n");
        		ble_cmd_attclient_attribute_write(msg->connection, ota_ctrl_handle, 1, &OTA_START);
        		change_state(state_reset_to_dfu);

        		// next, we expect the target to close connection and reboot into DFU mode
        	}
        	else
        	{
        		// target device is in OTA state. try to read version information (if available) before starting the actual upload
        		if(ota_vers_handle)
        		{
        			change_state(state_read_version);
        			printf("Reading version info...\n");
        			ble_cmd_attclient_read_by_handle(msg->connection, ota_vers_handle);
        		}
        		else
        		{
        			change_state(state_starting_ota);
        			printf("Starting OTA\n");
        			ble_cmd_attclient_attribute_write(msg->connection, ota_ctrl_handle, 1, &OTA_START);
        		}
        	}
        }
    }
    else if(state == state_read_version)
    {
    	// procedure_completed event is raised only if reading version information failed for some reason
    	printf("reading version info failed, code: %x\n", msg->result);
    	ble_cmd_connection_disconnect(conn_handle);
    	change_state(state_ota_disconnect);
    }
    else if(state == state_starting_ota)
    {

    	change_state(state_ebl_upload);

    	// start sending EBL data
    	printf("START UPLOAD\n");
    	dfu_start_time = time(NULL);
       	send_ebl_data(0);

    }
    else if (state == state_ota_finish)
    {
    	printf("OTA finished. result code: %x\n", msg->result);

    	if(msg->result != 0)
    	{
    		printf("############################\n");
    		printf("### ERROR: writing 0x03 to ota_control returned code %x = ", msg->result);
    		if(msg->result == 0x0480)
    		{
    			printf("CRC error");
    		}
    		else
    		{
    			printf("???");
    		}
    		printf("\n############################\n");
    	}
    	// as a last step, close the connection to target:
    	ble_cmd_connection_disconnect(conn_handle);
    	change_state(state_ota_disconnect);
    }
    else if (state == state_reset_to_dfu)
    {
    	printf("reset_to_dfu - got procedure completed\n");
    }
}

void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
	int i;

	if(msg->uuid.len == 16)
	{
		if(memcmp(msg->uuid.data, OTA_CTRL_UUID, 16)==0)
		{
			ota_ctrl_handle = msg->chrhandle;
			printf("OTA_CTRL char found, handle %d\n", ota_ctrl_handle);
		}
		else if(memcmp(msg->uuid.data, OTA_DATA_UUID, 16)==0)
		{
			ota_data_handle = msg->chrhandle;
			printf("OTA_DATA char found, handle %d\n", ota_data_handle);
		}
		else if(memcmp(msg->uuid.data, STACK_VERSION_UUID, 16)==0)
		{
			sdk_vers_handle = msg->chrhandle;
			printf("SDK_VERSION char found, handle %d\n", sdk_vers_handle);
		}
		else if(memcmp(msg->uuid.data, OTA_VERSION_UUID, 16)==0)
		{
			ota_vers_handle = msg->chrhandle;
			printf("OTA_VERSION char found, handle %d\n", ota_vers_handle);
		}
		else if(memcmp(msg->uuid.data, BLD_VERSION_UUID, 16)==0)
		{
			bld_vers_handle = msg->chrhandle;
			printf("BLD_VERSION char found, handle %d\n", bld_vers_handle);
		}
		else if(memcmp(msg->uuid.data, APP_VERSION_UUID, 16)==0)
		{
			app_vers_handle = msg->chrhandle;
			printf("APP_VERSION char found, handle %d\n", app_vers_handle);
		}
		else
		{
		    printf("Unknown characteristic UUID:");
		    for(i=0;i<16;i++)
		    {
		    	printf("%2.2x_", msg->uuid.data[i]);
		    }
		    printf("\n");
		}
	}

}

typedef struct
{
	uint16 maj;
	uint16 min;
	uint16 patch;
	uint16 build;
} tsSdkVersion;

tsSdkVersion sSdkVersion;

uint32 appVersion;

/* struct representing the BLD verison. note: byte order is different compared to SDK / AppLoader version
 *
 *  example: value 00000401 indicates that the Gecko Bootloader version is “1.4” and the customer-specific part is 0x0000
 *
 * */
typedef struct
{
	uint16 appSpecific;
	uint8 min;
	uint8 maj;
} tsBldVersion;

tsBldVersion sBldVersion;

void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{
	int i;
	uint16 next_handle = 0;

	if(state != state_read_version)
	{
		printf("unexpected ble_evt_attclient_attribute_value event? handle %d\r\n", msg->atthandle);
		return;
	}

#ifdef DEBUG
	printf("attribute_value: handle %d\r\n", msg->atthandle);
	printf("value: ");
	for(i=0;i<msg->value.len;i++)
	{
		printf("%2.2x ", msg->value.data[i]);
	}
	printf("\r\n");
#endif

	if(msg->atthandle == ota_vers_handle)
	{
		ota_version = msg->value.data[0];
		printf("OTA version: %d\n", ota_version);
		next_handle = sdk_vers_handle;
	}
	else if(msg->atthandle == sdk_vers_handle)
	{
		memcpy(&sSdkVersion, msg->value.data, 8);
		printf("SDK version: %d.%d.%d-%d\n", sSdkVersion.maj,sSdkVersion.min, sSdkVersion.patch, sSdkVersion.build);
		next_handle = bld_vers_handle;
	}
	else if(msg->atthandle == bld_vers_handle)
	{
		memcpy(&sBldVersion, msg->value.data, 4);
		printf("Bootloader version: %d.%d (0x%4.4x)\n", sBldVersion.maj, sBldVersion.min, sBldVersion.appSpecific);
		next_handle = app_vers_handle;
	}
	else if(msg->atthandle == app_vers_handle)
	{
		memcpy(&appVersion,msg->value.data,4);
		printf("Application version: 0x%4.4x\n", appVersion);
		next_handle = 0;
	}
	else
	{
		printf("unexpected handle: %d\r\n", msg->atthandle);
		return;
	}

	if(next_handle)
	{
		// read next version information field from the remote GATT
		ble_cmd_attclient_read_by_handle(msg->connection, next_handle);
	}
	else
	{
		// no more version info -> start OTA
		if(0)
		{
		    	ble_cmd_connection_disconnect(conn_handle);
		    	change_state(state_ota_disconnect);
		}
		else
		{
			const uint8 OTA_START = 0x00;
			change_state(state_starting_ota);
			printf("Starting OTA\n");
			ble_cmd_attclient_attribute_write(msg->connection, ota_ctrl_handle, 1, &OTA_START);
		}
	}

}

void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{

    //change_state(state_disconnected);
    printf("Connection terminated. Reason: %4.4x\n", msg->reason);

    ota_handles_start = 0;
    ota_handles_end = 0;
    ota_ctrl_handle = 0;
    ota_data_handle = 0;
    sdk_vers_handle = 0;
    ota_vers_handle = 0;
    bld_vers_handle = 0;
    app_vers_handle = 0;

    switch(msg->reason)
    {
    	case 0x023E: printf("-> Connection Failed to be Established\n"); break;
    	case 0x0216: printf("-> Connection Terminated by Local Host\n"); break;
    	case 0x0208: printf("-> Connection Timeout\n"); break;
    	case 0x0213: printf("-> Remote User Terminated Connection\n"); break;
    	default: printf("\n");
    }

    if ( state == state_reset_to_dfu)
    {
    	printf("Got expected disconnect event, target is now rebooting into DFU mode.\n");
    	printf("-> open a new connection\n");
		// remote device is booting into OTA mode
        change_state(state_connecting);
        ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 6, 60, 100,0);
    }
    else
    {
    	change_state(state_finish);
    }

    conn_status = 0;

}

void ble_rsp_attclient_attribute_write(const struct ble_msg_attclient_attribute_write_rsp_t *msg)
{
	printf("WRITE resp: %x\n", msg->result);

	switch (state)
	{
		case state_ota_finish:
		{
			if(msg->result == 0x0182)
			{
				uint8 OTA_FINISH = 0x03;
				printf("OTA finish: retry writing to OTA ctrl\n");
				ble_cmd_attclient_attribute_write(conn_handle, ota_ctrl_handle, 1, &OTA_FINISH);
			}
			else if (msg->result != 0)
			{
				printf("OTA finish: write failed with code %2.2x\n", msg->result);
				change_state(state_finish);
			}
		}
		break;

		default:
			printf("state %s: write resp = %2.2x\n", state_names[state], msg->result);
			break;
	}
}

void ble_rsp_attclient_read_by_handle(const struct ble_msg_attclient_read_by_handle_rsp_t *msg)
{
	if(msg->result)
	{
		printf("ERROR: ble_rsp_attclient_read_by_handle: %x\r\n", msg->result);
	}
}

void ble_rsp_attclient_write_command(const struct ble_msg_attclient_write_command_rsp_t *msg)
{
	static int progress_prev = 0;
	int progress;
	time_t t_now;

	if(state != state_ebl_upload)
	{
		printf("unexpected ble_rsp_attclient_write_command\r\n");
		return;
	}

	if(msg->result == 0x0182)
	{
		// try again
		send_ebl_data(1);
	}
	else if(msg->result)
	{
		printf("ERROR: write_command failed with code:%x\n", msg->result);
		change_state(state_finish);
	}
	else
	{
		dfu_current_pos += dfu_size;
		dfu_toload -= dfu_size;

		t_now = time(NULL);
		if (t_now != dfu_start_time && dfu_total>0)
		{
			progress = (int)(100 * dfu_current_pos / dfu_total);

			if(progress != progress_prev)
			{
				printf("%d%% %.2fkbit/s\n",
					progress,
					dfu_current_pos * 8.0 / 1000.0 / difftime(t_now, dfu_start_time));
				progress_prev = progress;
			}
		}

		if(dfu_toload > 0)
		{
			send_ebl_data(0);
		}
		else
		{
			uint8 OTA_FINISH = 0x03;
			// all data sent.
			printf("EBL file upload complete\n");

			// finish OTA
			  printf("Finishing OTA\n");

			  change_state(state_ota_finish);
			  ble_cmd_attclient_attribute_write(conn_handle, ota_ctrl_handle, 1, &OTA_FINISH);

		}
	}

}


/* soft timer callback */
void ble_evt_hardware_soft_timer(const struct ble_msg_hardware_soft_timer_evt_t *msg)
{

	if(scan_countdown > 0)
	{
		scan_countdown--;

		if(scan_countdown == 0)
		{
			printf("Stopping scan.\n");
			// stop the soft timer first:
			ble_cmd_hardware_set_soft_timer(0, 0, 0);
			// stop scan:
			ble_cmd_gap_end_procedure();
			exit(1);
		}
	}

}

int main(int argc, char *argv[]) {
    char *uart_port = "";
    ApplicationProperties_t props;
    //dump_init();
    
    // Not enough command-line arguments
    if (argc <= CLARG_PORT) {
        usage(argv[0]);
        return 1;
    }

    // COM port argument
    if (argc > CLARG_PORT) {
        if (strcmp(argv[CLARG_PORT], "list") == 0) {
            uart_list_devices();
            return 1;
        }
        else {
            uart_port = argv[CLARG_PORT];
        }
    }

    // Action argument
    if (argc > CLARG_ACTION) {
        int i;
        for (i = 0; i < strlen(argv[CLARG_ACTION]); i++) {
            argv[CLARG_ACTION][i] = tolower(argv[CLARG_ACTION][i]);
        }

        if (strcmp(argv[CLARG_ACTION], "scan") == 0) {
            action = action_scan;
            scan_countdown = 50; // scan for 5 seconds and then exit
        }
        else if (strcmp(argv[CLARG_ACTION], "info") == 0) {
            action = action_info;
        }
        else {
            int i;
            short unsigned int addr[6];
            if (sscanf(argv[CLARG_ACTION],
                    "%02hx:%02hx:%02hx:%02hx:%02hx:%02hx",
                    &addr[5],
                    &addr[4],
                    &addr[3],
                    &addr[2],
                    &addr[1],
                    &addr[0]) == 6) {

                for (i = 0; i < 6; i++) {
                    connect_addr.addr[i] = addr[i];
                }
                action = action_connect;
            }
        }
    }
    if (action == action_none) {
        usage(argv[0]);
        return 1;
    }


    //filename
    if(action == action_connect)
    {
    	printf("trying to open EBL file %s\n", argv[CLARG_OPTIONS]);
    	dfu_file = fopen(argv[CLARG_OPTIONS], "rb");
    	if (dfu_file == NULL)
    	{
    		printf("cannot open file %s\n",argv[CLARG_OPTIONS]);
    		exit(-1);
    	}

    	if(dfu_read_size() < 0)
    	{
    		exit(-1);
    	}
    }


    bglib_output = output;

    if(dfu_file)
    {
    	// try to parse some information from the EBL file:
    	find_app_properties(dfu_file, &props);

    	fseek(dfu_file, 0L, SEEK_SET); // rewind back to start
    }

    if (uart_open(uart_port)) {
        printf("ERROR: Unable to open serial port\n");
        return 1;
    }

    // Reset dongle to get it into known state
    ble_cmd_system_reset(0);
    uart_close();
    do {
        usleep(500000); // 0.5s
    } while (uart_open(uart_port));

    // Execute action
    if (action == action_scan) {
        ble_cmd_gap_discover(gap_discover_observation);

        // start a soft timer with 10 ms interval
        ble_cmd_hardware_set_soft_timer(3200, 0, 0);

    }
    else if (action == action_info) {
        ble_cmd_system_get_info();
    }
    else if (action == action_connect) {
        printf("Trying to connect\n");
        change_state(state_connecting);
        ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 6, 60, 100,0);
    }

    // Message loop
    while (state != state_finish) {
        if (read_message(UART_TIMEOUT) > 0) break;
    }

    uart_close();

    return 0;
}
