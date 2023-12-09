void motor_init();
void set_speed(float new_speed); //mm/sec
void move(float x, float y);  //mm
void home(); //mm/sec
void wait(int ms);
void servo_angle_set(float angle);
void start_extrusion();
void stop_extrusion();
void extrude();
void reset_extruder();
