
#define bufferLength 				10
#define filterWindowLength 	100	// moving average buffer length
#define MENU_STRING_LENGTH 	50
#define menu_list_length   	30	//number of all pages of menu
#define MENU_DEPTH					30	// maximum menu number that you can go inside(for active menu stack)
#define eeprom_length 			30	//number of addresses in the eeprom (each is int32_t variable)

#define p1_p_eeprom_add  				(uint32_t)0
#define p2_p_eeprom_add  				(uint32_t)1
#define p1_t_eeprom_add  				(uint32_t)2
#define p2_t_eeprom_add  				(uint32_t)3
#define pid_p_eeprom_add  			(uint32_t)4
#define pid_i_eeprom_add  			(uint32_t)5
#define pid_d_eeprom_add 			 	(uint32_t)6
#define relay_max_eeprom_add  	(uint32_t)7
#define relay_min_eeprom_add  	(uint32_t)8
#define controller_on_off_eeprom_add  	(uint32_t)9
#define controller_type_eeprom_add  		(uint32_t)10
#define controller_setpoint_eeprom_add  (uint32_t)11
#define ph_calibration_temp_add  				(uint32_t)12
#define ATC_eeprom_add 									(uint32_t)13
#define REL_FUNC_1_EEPROM_ADD 					(uint32_t)14
#define REL_FUNC_2_EEPROM_ADD 					(uint32_t)15
#define REL_FUNC_3_EEPROM_ADD 					(uint32_t)16
#define REL_FUNC_4_EEPROM_ADD 					(uint32_t)17
#define STABILIZATION_TIME_EEPROM_ADD 	(uint32_t)18
#define STABILIZATION_RANGE_EEPROM_ADD 	(uint32_t)19

#define float_to_int_factor  100000.0f



#define relay1_func  menu_list[17].values[0]
#define relay2_func  menu_list[17].values[1]
#define relay3_func  menu_list[17].values[2]
#define relay4_func  menu_list[17].values[3]
#define supply_relay_state  menu_list[18].values[0] 
#define drain_relay_state  menu_list[18].values[1] 
#define wash_relay_state  menu_list[18].values[2] 
#define kcl_relay_state  menu_list[18].values[3] 
#define pump_func_num 0
#define supply_func_num 1
#define drain_func_num 2
#define wash_func_num 3
#define kcl_func_num 4
#define manual_wash_menu 18
#define Auto_Wash_Menu 20
#define AUTO_WASH_STATE_MENU 21

#define drain1_func_time 					menu_list[20].values[0] 
#define wash_func_time  					menu_list[20].values[1] 
#define drain2_func_time					menu_list[20].values[2] 
#define kcl_func_time 						menu_list[20].values[3]

#define ATC 											menu_list[22].values[0]

#define AWS_Supply_State  				menu_list[21].values[0] 
#define AWS_Drain_State 					menu_list[21].values[1] 
#define AWS_KCl_State  						menu_list[21].values[2] 
#define AWS_Wash_State  					menu_list[21].values[3] 

#define CONTROLLER_THRESHOLD_MAX	menu_list[9].values[0]
#define CONTROLLER_THRESHOLD_MIN	menu_list[9].values[1]


#define CONTROLLER_ON_OFF 				menu_list[8].values[0]
#define CONTROLLER_TYPE 					menu_list[8].values[1]
#define CONTROLLER_SETPOINT 			menu_list[8].values[3]

#define PID_P 										menu_list[10].values[0]
#define PID_I 										menu_list[10].values[1]
#define PID_D 										menu_list[10].values[2]

#define HOUR 											menu_list[11].values[0]
#define MINUTE 										menu_list[11].values[1]
#define DATE 											menu_list[11].values[2]
#define MONTH 										menu_list[11].values[3]
#define YEAR 											menu_list[11].values[4]

#define STABILIZATION_TIME 				menu_list[2].values[1]
#define STABILIZATION_RANGE 			menu_list[2].values[2]

