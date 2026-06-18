/**
 ******************************************************************************
 * @file    ATCmdParser.c
 * @author  ele
 * @version V1.0.0
 * @date    2023-2-9
 * @brief   DX2002 module AT command parser
 ******************************************************************************
 *
 * Copyright (c) 2019-2028 QingYing Co.,Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

#include "ATCmdParser.h"
#include "ble_serial.h"
#include "mx_common.h"
#include "rtthread.h"
#include <stdio.h>
#include "global_conf.h"

#define LOG_TAG "at_parse"
#define LOG_LVL LOG_LVL_INFO //LOG_LVL_DBG
#include <ulog.h>

/******************************************************************************
 *                                 Constants
 ******************************************************************************/

/******************************************************************************
 *                              Variable Definitions
 ******************************************************************************/

#define AT_BUFFER_SIZE                    (512)         /**< Max buffer used for AT command analyser, no used for recv raw data */
static struct oob* AT_oobs;
static char AT_buffer[AT_BUFFER_SIZE];
static const char* _output_delimiter;
static int _output_delim_size;
static const char* _input_delimiter;
static int _input_delim_size;
static char at_mode=0;
static int  _timeout = 100;   // 默认100ms

/******************************************************************************
 *                              Function Definitions
 ******************************************************************************/

bool ATCmdParser_vrecv(const char* response, va_list args)
{
    char _in_prev = 0;
    bool _aborted;
restart:
    _aborted = false;
    // Iterate through each line in the expected response
    while (response[0]) {
        // Since response is const, we need to copy it into our buffer to
        // add the line's null terminator and clobber value-matches with asterisks.
        //
        // We just use the beginning of the buffer to avoid unnecessary allocations.
        int i = 0;
        int offset = 0;
        bool whole_line_wanted = false;

        while (response[i]) {
            if (response[i] == '%' && response[i + 1] != '%' && response[i + 1] != '*') {
                AT_buffer[offset++] = '%';
                AT_buffer[offset++] = '*';
                i++;
            } else {
                AT_buffer[offset++] = response[i++];
                // Find linebreaks, taking care not to be fooled if they're in a %[^\n] conversion specification
                if (response[i - 1] == '\n' && !(i >= 3 && response[i - 3] == '[' && response[i - 2] == '^')) {
                    whole_line_wanted = true;
                    break;
                }
            }
        }

        // Scanf has very poor support for catching errors
        // fortunately, we can abuse the %n specifier to determine
        // if the entire string was matched.
        AT_buffer[offset++] = '%';
        AT_buffer[offset++] = 'n';
        AT_buffer[offset++] = 0;

//		LOG_D("AT? %s",AT_buffer);
        // To workaround scanf's lack of error reporting, we actually
        // make two passes. One checks the validity with the modified
        // format string that only stores the matched characters (%n).
        // The other reads in the actual matched values.
        //
        // We keep trying the match until we succeed or some other error
        // derails us.
        int j = 0, dummy = 0;
        int dummy_pos[20];
//		rt_thread_delay(10);
        while (true) {
            // Receive next character
            int c = ble_serial_getc_timeout(100);
//					int c = ble_serial_getc_nowait();					
            if (c < 0) {
if(enableDebug){
							LOG_D("AT(Timeout)");
}
                return false;
            }

            /* Possible not existed string may cause %n function failed:
			example: "cmd:%*s\r\n%n" not match "cmd:\r\n", %n will not 
			give a valid value, so we give some dummy chars */
            if (c == '\n' && _in_prev == ':') {
                dummy_pos[dummy++] = j;
                AT_buffer[offset + j++] = 'x';
            }

            _in_prev = c;

            AT_buffer[offset + j++] = c;
            AT_buffer[offset + j] = 0;

//            struct oob* oob = AT_oobs;
//            // Check for oob data
//            for (; oob; oob = (struct oob*)oob->next) {
//                if ((unsigned)j == oob->len && memcmp(oob->prefix, AT_buffer + offset, oob->len) == 0) {
//if(enableDebug){
//									LOG_D("AT! %s", oob->prefix);
//}
//					oob->cb();

//                    if (_aborted) {
//						LOG_D("AT(Aborted)");
//                        return false;
//                    }
//                    // oob may have corrupted non-reentrant buffer,
//                    // so we need to set it up again
//                    goto restart;
//                }
//            }

            // Check for match
            int count = -1;
            if (whole_line_wanted && c != '\n') {
                // Don't attempt scanning until we get delimiter if they included it in format
                // This allows recv("Foo: %s\n") to work, and not match with just the first character of a string
                // (scanf does not itself match whitespace in its format string, so \n is not significant to it)
            } else {
                sscanf(AT_buffer + offset, AT_buffer, &count);   // %n为统计字符串数量
//				LOG_D("AT< %s, %d,%d", AT_buffer + offset, count, j);
			}

            // We only succeed if all characters in the response are matched
            if (count == j) {
                /* Remove dummy chars*/
                for (int k = 0; k < dummy; k++, j--) {
                    dummy_pos[k] -= k;
                    memcpy(AT_buffer + offset + dummy_pos[k], AT_buffer + offset + dummy_pos[k] + 1, j - dummy_pos[k] - 1);
if(enableDebug){
									LOG_D("AT= %s\n", AT_buffer + offset);
}
                }
								if(enableDebug){
								LOG_D("AT= %s\n", AT_buffer + offset);
								}
                // Reuse the front end of the buffer
                memcpy(AT_buffer, response, i);
                AT_buffer[i] = 0;

                // Store the found results
                vsscanf(AT_buffer + offset, AT_buffer, args);

                // Jump to next line and continue parsing
                response += i;
                break;
            }

            // Clear the buffer when we hit a newline or ran out of space
            // running out of space usually means we ran into binary data
            if (c == '\n' || j + 1 >= AT_BUFFER_SIZE - offset) {
	//			LOG_D("AT< %s\n", AT_buffer + offset);
                j = 0;
                dummy = 0;
            }
        }
    }

    return true;
}

// Command parsing with line handling
bool ATCmdParser_vsend(const char* command, va_list args)
{
//	while (ATCmdParser_process_oob())
//			;
//		rt_kprintf(">>>>>>>>>>>\r\n");
    // Create and send command
    if (vsprintf(AT_buffer, command, args) < 0) {
        return false;
    }

    for (int i = 0; AT_buffer[i]; i++) {
        if (ble_serial_putc(AT_buffer[i]) < 0) {
            return false;
        }
    }
    // Finish with newline
    for (size_t i = 0; _output_delimiter[i]; i++) {
        if (ble_serial_putc(_output_delimiter[i]) < 0) {
            return false;
        }
    }
//if(enableDebug){
//	LOG_D("AT> %s", AT_buffer);
//}
//rt_kprintf("-------\r\n");
    return true;
}

// 不带回车换行
bool ATCmdParser_vsend_start(const char* command, va_list args)
{
//    while (ATCmdParser_process_oob())
//        ;
    // Create and send command
    if (vsprintf(AT_buffer, command, args) < 0) {
        return false;
    }

    for (int i = 0; AT_buffer[i]; i++) {
        if (ble_serial_putc(AT_buffer[i]) < 0) {
            return false;
        }
    }
//if(enableDebug){	
//	LOG_D("AT> %s", AT_buffer);
//}
    return true;
}

// 回车换行独立函数 
bool ATCmdParse_vsendEnd(void)
{
    // Finish with newline
    for (size_t i = 0; _output_delimiter[i]; i++) {
        if (ble_serial_putc(_output_delimiter[i]) < 0) {
            return false;
        }
    }
	return true;
}

bool ATCmdParser_recv(const char* response, ...)
{
    va_list args;
    va_start(args, response);
    bool res = ATCmdParser_vrecv(response, args);
    va_end(args);
    return res;
}

bool ATCmdParser_send(const char* command, ...)
{
    va_list args;
    va_start(args, command);
    bool res = ATCmdParser_vsend(command, args);
    va_end(args);
    return res;
}

bool ATCmdParser_sendStart(const char* command, ...)
{
    va_list args;
    va_start(args, command);
    bool res = ATCmdParser_vsend_start(command, args);
    va_end(args);
    return res;
}

// read/write handling with timeouts
int ATCmdParser_write(const char* data, int size)
{
    int i = 0;
    for (; i < size; i++) {
        if (ble_serial_putc(data[i]) < 0) {
            return -1;
        }
        //rt_thread_delay(1);
    }
    return i;
}

int ATCmdParser_read(char* data, int size)
{
    int i = 0;
    for (; i < size; i++) {
        int c = ble_serial_getc_timeout(_timeout);
        if (c < 0) {
            return -1;
        }
        data[i] = c;
    }
    return i;
}

void ATCmdParser_add_oob(const char* prefix, oob_callback cb)
{
    struct oob* oob;
     oob= (struct oob*) rt_malloc(sizeof(struct oob));
	 if (oob == NULL) {
		 rt_kprintf("oob No Enough Memory\n");
		 return;
	 }

    oob->len = strlen(prefix);
    oob->prefix = prefix;
    oob->cb = cb;
    oob->next = AT_oobs;
    AT_oobs = oob;
}

// 0:处理通讯协议包 1:处理AT指令集
void ATCmdParser_set_mode(char mode)
{
	at_mode = mode;
}

bool ATCmdParser_process_oob(void)
{
		if(at_mode){
			return false;
		}

    int i = 0;
    while (true) {
        // Receive next character
        int c = ble_serial_getc_timeout(100);
        if (c < 0) {
            return false;
        }
        AT_buffer[i++] = c;
        AT_buffer[i] = 0;

        // Check for oob data
        struct oob* oob = AT_oobs;
        while (oob) {
            if (i == (int)oob->len && memcmp(oob->prefix, AT_buffer, oob->len) == 0) {
							if(enableDebug){
				LOG_D("OOB! %s", oob->prefix);
							}
                oob->cb();
                return true;
            }
            oob = oob->next;
        }

        // Clear the buffer when we hit a newline or ran out of space
        // running out of space usually means we ran into binary data
        if (i + 1 >= AT_BUFFER_SIZE || strcmp(&AT_buffer[i - _input_delim_size], _input_delimiter) == 0) 
		{
			if(enableDebug){
			LOG_D("AT< %s, %d", AT_buffer, i);
			}
            i = 0;
        }
    }
}

int ATCmdParser_analyse_args(char args[], char* arg_list[], int list_size)
{
    char _in_prev = 0;
    int arg_num = 1;
    size_t len = strlen(args);
    arg_list[0] = args;
if(enableDebug){
	LOG_D( "check=%s,len=%dn", args, len);
}
    for (int i = 0; i <= len; i++) {		
        if (args[i] == ',') {
					if(enableDebug){
			LOG_D("find ,\n");
					}
            if (_in_prev == '\\') {
                args[i - i] = ',';
                memcpy(&args[i], &args[i + 1], len - (i + 1));
            } else {
                args[i] = 0;
                arg_list[arg_num++] = &args[i + 1];
                if (arg_num > list_size)
                    break;
            }
        }
        _in_prev = args[i];
    }

	for(int i=0;i<arg_num;i++)
	{
		if(enableDebug){
		LOG_D( "arg_list[%d]=%s",i,arg_list[i]);
		}
	}
    return arg_num;
}

void ATCmdParser_init(const char* output_delimiter, const char* input_delimiter, int timeout)
{
    _output_delimiter = output_delimiter;
    _output_delim_size = strlen(output_delimiter);

    _input_delimiter = input_delimiter;
    _input_delim_size = strlen(input_delimiter);

    ble_serial_init("uart5", timeout); 
		_timeout = timeout;
}
