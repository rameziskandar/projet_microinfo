

void avoid_obstacle(uint16_t distance);

void turn_right(uint16_t turn_time);

void turn_left(uint16_t turn_time);

void go_straight(uint16_t time_forward);

uint16_t convert_angle(float angle);

uint16_t convert_distance(int16_t distance);

uint16_t straight_avoid(float component_x, float component_y, float angle, uint16_t distance);
