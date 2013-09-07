#include "gcode.h"

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>


#include "stepper.h"
#include "temp.h"

// Enums
typedef enum
{
    millimiters = 0,
    inches,
} UNIT;


typedef enum
{
    absolute = 0,
    relative,
}MOVEMODE;

// Structures
typedef struct
{
    // Parameters
    UNIT units;
    MOVEMODE extruderMode;

    // Peripherals state
    bool spindleOn; /// \todo to add : coolant, vacuum
    float extruderTempOrder;
    float bedTempOrder;

    /// \todo to be complete with all other commands
} gcode_state;


// The IEEE 754 standard specifies a binary32 as having:
// Sign bit: 1 bit
// Exponent width: 8 bits
// Significand precision: 24 (23 explicitly stored) => 2^24 = 16777216 coded without loss = 16 m @ 1µm precision.

typedef struct
{
    // pos
    float x;
    float y;
    float z;
    float e;

    // param
    float p; //
    float f; // feedrate, ex mm/s
    float s; // Speed, ex RPM

    // arc
    float i;
    float j;
    float k;

    // commands
    float g;
    float m;
    float t;

    // Line number
    float n;
    float checksum;
} cmd_param;


// Member variables
//static char receiveBuffer[256];
static gcode_state state =
{
    .units = millimiters,
    .extruderMode = absolute,
    .spindleOn = false,
    .extruderTempOrder = 100,
    .bedTempOrder = 50,

};

// Private Prototypes
void parseCode(const char* const data);
void processGCode(const cmd_param param);
void processMCode(const cmd_param param);
void processTCode(const cmd_param param);

// Definitions

void gcode_init()
{
    state.units = millimiters;
}

void gcode_parse(char* data)
{
    // Add buffering logic here
    parseCode(data);
}


void parseCode(const char* const data)
{
    // Get Letter and Number
    char letter = '\0';
    float value = 0;

    cmd_param param;
    memset(&param, 0, sizeof(param)); // Init to zero structure (float at 0 should be 0.0 value).


    char* chrPtr = &data[0];

//    puts("Start parsing\n");
    while ( (*chrPtr != '\0') && (chrPtr < (data + strlen(data))) ) // While not at the end of the string
    {
//        if(*chrPtr == ' ')
//        {
////            puts("Space, continue\n");
//            chrPtr++;
//            continue;
//        }

        if( ( (*chrPtr >= 'A')
              && (*chrPtr <= 'Z') )
                || (*chrPtr == '*') ) // Is a Valid GCode letter
        {
//            puts("Valid letter found\n");
            letter = *chrPtr;

            chrPtr++; // Go to next character. Should be a number or a space

            if (*chrPtr != '\0') // If it is not end of string
            {
//                puts("Value found\n");
                value = strtof(chrPtr, &chrPtr);

                // affect value to the parameter list
                switch(letter)
                {
                    case 'E': // E = Precision feedrate for threading on lathes
                        param.e = value;
                        break;
                    case 'F': // F = Defines feed rate
                        param.f = value;
                        break;
                    case 'G': //
                        param.g = value;
                        break;

                    case 'I': // I = Defines arc center in X axis for G02 or G03 arc commands.
                        param.i = value;
                        break;
                    case 'J': // J = Defines arc center in Y axis for G02 or G03 arc commands.
                        param.j = value;
                        break;
                    case 'K': // K = Defines arc center in Z axis for G02 or G03 arc commands.
                        param.k = value;
                        break;
                    case 'M': //
                        param.m = value;
                        break;
                    case 'N': //
                        param.n = value;
                        break;
                    case '*': //
                        param.checksum = value;
                        break;

                    case 'P': // P = Serves as parameter address for various G and M codes
                        param.p = value;
                        break;

                    case 'S': // S = Defines speed, either spindle speed or surface speed depending on mode
                        param.s = value;
                        break;
                    case 'T': //
                        param.t = value;
                        break;

                    case 'X': // X = Absolute or incremental position of X axis.
                        param.x = value;
                        break;
                    case 'Y': // Y = Absolute or incremental position of Y axis
                        param.y = value;
                        break;
                    case 'Z': // Z = Absolute or incremental position of Z axis
                        param.z = value;
                        break;
                    default:
                        //
                        break;
                }
//                break; /// \todo Debug line...

            }
//            puts("Next char\n");
        }
        else
        {
            chrPtr++;
        }
    }

//    puts("Finished parsing\n");

    //Dump the parameters read:
//    printf("e=%f\n", param.e);
//    printf("f=%f\n", param.f);
//    printf("g=%f\n", param.g);
//    printf("i=%f\n", param.i);
//    printf("j=%f\n", param.j);
//    printf("k=%f\n", param.k);
//    printf("m=%f\n", param.m);
//    printf("p=%f\n", param.p);
//    printf("s=%f\n", param.s);
//    printf("t=%f\n", param.t);
//    printf("x=%f\n", param.x);
//    printf("y=%f\n", param.y);
//    printf("z=%f\n", param.z);


    if (param.g != 0)
    {
        processGCode(param);
    }
    else if (param.m != 0)
    {
        processMCode(param);
    }
    else if (param.t != 0)
    {
        processTCode(param);
    }
/*
    sscanf(data, "%c%d", &letter, &num);

    switch (letter)
    {
        case 'A': // A = Absolute or incremental position of A axis (rotational axis around X axis)
        case 'B': // B = Absolute or incremental position of B axis (rotational axis around Y axis)
        case 'C': // C = Absolute or incremental position of C axis (rotational axis around Z axis)
        case 'D': // D = Defines diameter or radial offset used for cutter compensation. D is used for depth of cut on lathes.
        case 'E': // E = Precision feedrate for threading on lathes
        case 'F': // F = Defines feed rate
            break;
        case 'G': // G = Address for preparatory commands
            processGCode(param.g, param);
            break;
        case 'H': // H = Defines tool length offset;
        case 'I': // I = Defines arc center in X axis for G02 or G03 arc commands.
        case 'J': // J = Defines arc center in Y axis for G02 or G03 arc commands.
        case 'K': // K = Defines arc center in Z axis for G02 or G03 arc commands.
        case 'L': // L = Fixed cycle loop count;
            break;
        case 'M': // M = Miscellaneous function
            processMCode(num, data);
            break;
        case 'N': // N = Line (block) number in program;
        case 'O': // O = Program name
        case 'P': // P = Serves as parameter address for various G and M codes
        case 'Q': // Q = Peck increment in canned cycles
        case 'R': // R = Defines size of arc radius, or defines retract height in milling canned cycles
        case 'S': // S = Defines speed, either spindle speed or surface speed depending on mode
            break;
        case 'T': // T = Tool selection
            processTCode(num);
            break;
        case 'U': // U = Incremental axis corresponding to X axis (typically only lathe group A controls)
        case 'V': // V = Incremental axis corresponding to Y axis
        case 'W': // W = Incremental axis corresponding to Z axis (typically only lathe group A controls)
        case 'X': // X = Absolute or incremental position of X axis.
        case 'Y': // Y = Absolute or incremental position of Y axis
        case 'Z': // Z = Absolute or incremental position of Z axis
        default:
            break;
    }
    */
}

void processGCode(const cmd_param param)
{
    switch ((int)param.g)
    {
        case 0: // G0 = Rapid move
            puts("ok\n");
            break;
        case 1: // G1 = Controlled move
            if ( (param.x != 0.0) || (param.y != 0.0) )
            {
                stepper_move(param.x, param.y);
            }
//            if ( param.x != 0.0 )
//            {
//                stepper_move(param.x, param.y);
//            }
            else
            {
                puts("ok\n");
            }
            break;
        case 4: // G4 = Dwell
            puts("ok\n");
            break;
        case 10: // G10 = Head Offset
            puts("ok\n");
            break;
        case 20: // G20 = Set Units to Inches
            puts("ok\n");
            break;
        case 21: // G21 = Set Units to Millimeters
            puts("ok\n");
            break;
        case 28: // G28 = Move to Origin
            stepper_reset(); /// \todo change to real move to origin
            puts("ok\n");
            break;
        case 29:
        case 30:
        case 31:
        case 32: // G29-32 = Bed probing
            puts("ok\n");
            break;
        case 90: // G90 = Set to Absolute Positioning
            stepper_set_absolute();
            puts("ok\n");
            break;
        case 91: // G91 = Set to Relative Positioning
            stepper_set_relative();
            puts("ok\n");
            break;
        case 92: // G92 = Set Position
            puts("ok\n");
            break;
        default:
            puts("ok\n");
            break;
    }
}

void processMCode(const cmd_param param)
{
    switch((int)param.m)
    {
        case 0: // M0 = Stop
        case 1: // M1 = Sleep
            puts("ok\n");
            break;
        case 3: // M3 = Spindle On, Clockwise (CNC specific)
        case 4: // M4 = Spindle On, Counter-Clockwise (CNC specific)
            state.spindleOn = true;
            puts("ok\n");
            break;
        case 5: // M5 = Spindle Off (CNC specific)
            state.spindleOn = false;
            puts("ok\n");
            break;
        case 7: // M7 = Mist Coolant On (CNC specific)
        case 8: // M8 = Flood Coolant On (CNC specific)
        case 9: // M9 = Coolant Off (CNC specific)
        case 10: // M10 = Vacuum On (CNC specific)
        case 11: // M11 = Vacuum Off (CNC specific)
        case 17: // M17 = Enable/Power all stepper motors
        case 18: // M18 = Disable all stepper motors
        case 20: // M20 = List SD card
        case 21: // M21 = Initialize SD card
        case 22: // M22 = Release SD card
        case 23: // M23 = Select SD file
        case 24: // M24 = Start/resume SD print
        case 25: // M25 = Pause SD print
        case 26: // M26 = Set SD position
        case 27: // M27 = Report SD print status
        case 28: // M28 = Begin write to SD card
        case 29: // M29 = Stop writing to SD card
        case 30: // M30 = Delete a file on the SD card
        case 40: // M40 = Eject
        case 41: // M41 = Loop
        case 42: // M42 = Stop on material exhausted / Switch I/O pin
        case 43: // M43 = Stand by on material exhausted
        case 80: // M80 = ATX Power On
        case 81: // M81 = ATX Power Off
        case 82: // M82 = set extruder to absolute mode
        case 83: // M83 = set extruder to relative mode
        case 84: // M84 = Stop idle hold
        case 92: // M92 = Set axis_steps_per_unit
        case 98: // M98 = Get axis_hysteresis_mm
        case 99: // M99 = Set axis_hysteresis_mm
        case 101: // M101 = Turn extruder 1 on Forward / Undo Extruder Retraction
        case 102: // M102 = Turn extruder 1 on Reverse
        case 103: // M103 = Turn all extruders off / Extruder Retraction
        case 104: // M104 = Set Extruder Temperature
            //sscanf(data, "M104 S%d", &state.extruderTempOrder);
            state.extruderTempOrder = param.s;
            puts("ok\n");
            break;
        case 105: // M105 = Get Extruder Temperature
//            printf("ok T:%d B:%d\n", temp_get_extruder(), state.bedTempOrder);
            printf("ok T:%.2f B:%.2f\n", state.extruderTempOrder, state.bedTempOrder);
            break;
        case 106: // M106 = Fan On
        case 107: // M107 = Fan Off
        case 108: // M108 = Set Extruder Speed
        case 109: // M109 = Set Extruder Temperature and Wait
        case 110: // M110 = Set Current Line Number
        case 111: // M111 = Set Debug Level
        case 112: // M112 = Emergency Stop
        case 113: // M113 = Set Extruder PWM
        case 114: // M114 = Get Current Position
        case 115: // M115 = Get Firmware Version and Capabilities
        case 116: // M116 = Wait
        case 117: // M117 = Get Zero Position
        case 118: // M118 = Negotiate Features
        case 119: // M119 = Get Endstop Status
        case 120: // M120 = Push
        case 121: // M121 = Pop
        case 126: // M126 = Open Valve
        case 127: // M127 = Close Valve
        case 128: // M128 = Extruder Pressure PWM
        case 129: // M129 = Extruder pressure off
        case 130: // M130 = Set PID P value
        case 131: // M131 = Set PID I value
        case 132: // M132 = Set PID D value
        case 133: // M133 = Set PID I limit value
        case 134: // M134 = Write PID values to EEPROM
        case 136: // M136 = Print PID settings to host
            puts("ok\n");
            break;
        case 140: // M140 = Bed Temperature (Fast)
            //sscanf(data, "M140 S%d", &state.bedTempOrder);
            state.bedTempOrder = param.s;
            puts("ok\n");
            break;
        case 141: // M141 = Chamber Temperature (Fast)
        case 142: // M142 = Holding Pressure
        case 143: // M143 = Maximum hot-end temperature
        case 160: // M160 = Number of mixed materials
        case 190: // M190 = Wait for bed temperature to reach target temp
        case 200: // M200 = Set filament diameter / Get Endstop Status
        case 201: // M201 = Set max printing acceleration
        case 202: // M202 = Set max travel acceleration
        case 203: // M203 = Set maximum feedrate
        case 204: // M204 = Set default acceleration
        case 205: // M205 = advanced settings
        case 206: // M206 = set home offset
        case 207: // M207 = calibrate z axis by detecting z max length
        case 208: // M208 = set axis max travel
        case 209: // M209 = enable automatic retract
        case 220: // M220 = set speed factor override percentage
        case 221: // M221 = set extrude factor override percentage
        case 226: // M226 = Gcode Initiated Pause
        case 227: // M227 = Enable Automatic Reverse and Prime
        case 228: // M228 = Disable Automatic Reverse and Prime
        case 229: // M229 = Enable Automatic Reverse and Prime
        case 230: // M230 = Disable / Enable Wait for Temperature Change
        case 240: // M240 = Start conveyor belt motor / Echo off
        case 241: // M241 = Stop conveyor belt motor / echo on
        case 245: // M245 = Start cooler
        case 246: // M246 = Stop cooler
        case 300: // M300 = Play beep sound
        case 301: // M301 = Set PID parameters - Hot End
        case 303: // M303 = Run PID tuning
        case 304: // M304 = Set PID parameters - Bed
        case 420: // M420 = Set RGB Colors as PWM
        case 500: // M500 = stores paramters in EEPROM
        case 501: // M501 = reads parameters from EEPROM
        case 502: // M502 = reverts to the default "factory settings".
        case 503: // M503 = Print settings
        default:
            puts("ok\n");
            break;
    }
}

void processTCode(const cmd_param param)
{
    switch ((int)param.t)
    {
        //case : // T = Select Tool
        default:
            puts("ok\n");
            break;
    }
}

void gcode_setExtruderTempMeasure(float temp)
{
    /// \todo replace order by measure
    state.extruderTempOrder = temp;
}
