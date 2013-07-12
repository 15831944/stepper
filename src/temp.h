#ifndef TEMP_H
#define TEMP_H

void temp_init(void);

void temp_set_extruder(int temp);
void temp_set_bed(int temp);

int temp_get_extruder(void);
int temp_get_bed(void);


#endif // TEMP_H
