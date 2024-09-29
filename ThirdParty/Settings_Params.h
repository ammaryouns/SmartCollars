/**
 * @file   	Settings_Params.h
 * @author 	Hassan
 * @version	
 * @date	Aug 19, 2016
 * 
 * @brief   
 */

#ifndef SETTINGS_PARAMS_H_
#define SETTINGS_PARAMS_H_

#include <stdint.h>

/**
 * @defgroup Settings_Params_Codes_t			Settings Params typedef
 * @{
 */

typedef uint8_t   Settings_Param_Codes_t;			/*!< The settings parameter is currently has a size of 1 byte.
														This typedef is defined incase the size of parameter needs to
														be change later.  */

#define UPLOAD_TIME					((Settings_Param_Codes_t)1)
#define DCU_FW_VER					((Settings_Param_Codes_t)2)
#define CROPMATIX_FW_VER			((Settings_Param_Codes_t)3)
#define COWLAR_FW_VER				((Settings_Param_Codes_t)4)
#define DATA_POST_ADDRESS			((Settings_Param_Codes_t)5)
#define FW_UPDATE_ADDRESS			((Settings_Param_Codes_t)6)
#define MAX_NUM_NODES				((Settings_Param_Codes_t)7)
#define DATA_SEQUENCE_DCU			((Settings_Param_Codes_t)8)
#define DATA_SEQUENCE_COWLAR		((Settings_Param_Codes_t)9)
#define DATA_SEQUENCE_CROPMATIX 	((Settings_Param_Codes_t)10)

/** @}*/


static const Settings_Param_Codes_t
    SETTINGS_UPDATE_LIST[] =   { UPLOAD_TIME,
                                 DCU_FW_VER,
                                 CROPMATIX_FW_VER,
                                 COWLAR_FW_VER,
                                 DATA_POST_ADDRESS,
                                 FW_UPDATE_ADDRESS,
                                 MAX_NUM_NODES,
                                 DATA_SEQUENCE_DCU,
                                 DATA_SEQUENCE_COWLAR,
                                 DATA_SEQUENCE_CROPMATIX};



#ifdef __cplusplus
extern "C"{
#endif 


#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_PARAMS_H_ */
