#ifndef GCODE_H
#define GCODE_H

void gcode_init();
void gcode_parse(char* data);
void gcode_setExtruderTempMeasure(float temp);

#endif // GCODE_H
