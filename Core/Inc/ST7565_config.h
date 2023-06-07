/*
 * Copyright (C) Effman s.r.o. - All rights reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

/**
 * @file ST7565_config.h
 * @author Michal
 * @date 20. 12. 2017
 * @brief Config file for ST7565 LCD
 */

#ifndef ST7565_CONFIG_H_
#define ST7565_CONFIG_H_

#define ENABLE_PARTIAL_UPDATE 0  /**< Uncomment this if you want to speed up update of the pixels (only update changed pixels)*/
#define ST7565_STARTBYTES     0
#define USE_OWN_FONTS         1  /**< Set it to 1 when you want to use own fonts, or to 0 when internal fonts */

#endif /* ST7565_CONFIG_H_ */
