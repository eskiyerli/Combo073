/**
  ******************************************************************************
  * @file    lib_NDEF.h
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    20-November-2013
  * @brief   This file help to manage NDEF file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MMY-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_NDEF_H
#define __LIB_NDEF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* include file which match the HW configuration */	 
#include "lib_wrapper.h"	

/** @addtogroup NFC_libraries
  * @{
  */


/** @addtogroup lib_NDEF
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  NDEF type supported
  */
typedef enum 
{
	UNKNOWN_TYPE = 0,
  VCARD_TYPE,
	WELL_KNOWN_ABRIDGED_URI_TYPE,
	URI_SMS_TYPE,
	URI_GEO_TYPE,
	URI_EMAIL_TYPE,
	SMARTPOSTER_TYPE,
	URL_TYPE,
	TEXT_TYPE,
	BT_TYPE,
	/* list of "external type" known by this demo, other external type will be addressed as UNKNWON_TYPE */
	M24SR_DISCOVERY_APP_TYPE 
} NDEF_TypeDef;


#define SP_MAX_RECORD							3
/**
  * @brief  structure to store record information
  */
typedef struct
{
  uint8_t RecordFlags;
  uint8_t TypeLength;
	uint8_t PayloadLength3;
	uint8_t PayloadLength2;
	uint8_t PayloadLength1;
	uint8_t PayloadLength0;
	uint8_t IDLength;
	uint8_t Type[0xFF];
	uint8_t ID[0xFF];
	uint16_t PayloadOffset;
	uint32_t PayloadBufferAdd;    /* add where payload content has been stored */
	NDEF_TypeDef NDEF_Type;  /* to store identification ID for application */
	void *SPRecordStructAdd[SP_MAX_RECORD]; /*in case of smart poster array to store add of other sRecordInfo struct */
	uint8_t NbOfRecordInSPPayload;
}sRecordInfo;		 
	 	
/* Exported constants --------------------------------------------------------*/

/** @defgroup lib_NDEF_Exported_Constants
  * @{
  */

#define NDEF_SIZE_OFFSET					0
#define FIRST_RECORD_OFFSET				2	

#define RECORD_FLAG_FIELD					1	 
#define TYPE_LENGTH_FIELD					1
#define ID_LENGTH_FIELD						1	 

/** @defgroup lib_NDEF_Record_area_mask
  * @{
  */	
#define	MB_Mask  									((uint8_t)(0x80))
#define ME_Mask 									((uint8_t)(0x40))
#define CF_Mask 									((uint8_t)(0x20))
#define SR_Mask 									((uint8_t)(0x10))
#define IL_Mask 									((uint8_t)(0x08))
#define TNF_Mask 									((uint8_t)(0x07))
/**
  * @}
  */

/** @defgroup lib_NDEF_TNF_type
  * @{
  */
#define TNF_Empty									0x00
#define TNF_WellKnown							0x01
#define TNF_MediaType							0x02
#define TNF_AbsoluteURI						0x03
#define TNF_NFCForumExternal			0x04
#define TNF_Unknown				  			0x05
#define TNF_Unchanged							0x06
#define TNF_Reserved							0x07
/**
  * @}
  */

/** @defgroup lib_NDEF_well_known_string
  * @{
  */
#define AAR_TYPE_STRING													"android.com:pkg"	
#define AAR_TYPE_STRING_LENGTH	 								15

#define M24SR_DISCOVERY_APP_STRING  						"st.com:m24sr_discovery_democtrl"	 
#define M24SR_DISCOVERY_APP_STRING_LENGTH	 			31	

#define VCARD_TYPE_STRING												"text/vcard"	
#define VCARD_TYPE_STRING_LENGTH	 							10	 

#define XVCARD_TYPE_STRING											"text/x-vCard"	 
#define XVCARD_TYPE_STRING_LENGTH	 							12

#define SMART_POSTER_TYPE_STRING								"Sp"
#define SMART_POSTER_TYPE_STRING_LENGTH	 				2
	 
#define URI_TYPE_STRING													"U"
#define URI_TYPE_STRING_LENGTH									1
	 
#define SMS_TYPE_STRING													"sms:"	
#define SMS_TYPE_STRING_LENGTH	 								4

#define GEO_TYPE_STRING													"geo:"	
#define GEO_TYPE_STRING_LENGTH	 								4

#define URI_LATITUDE_END												","
#define URI_LATITUDE_END_LENGTH									1

#define EMAIL_TYPE_STRING                				"mailto:"
#define EMAIL_TYPE_STRING_LENGTH								7

#define URI_FIRST_DATA_END											"?"
#define URI_FIRST_DATA_END_LENGTH								1

#define SUBJECT_BEGIN_STRING										"subject="
#define SUBJECT_BEGIN_STRING_LENGTH							8	 

#define MESSAGE_BEGIN_STRING										"body="	 
#define MESSAGE_BEGIN_STRING_LENGTH							5	 

#define URI_SECOND_DATA_END											"&"
#define URI_SECOND_DATA_END_LENGTH							1

#define TEXT_TYPE_STRING												"T"
#define TEXT_TYPE_STRING_LENGTH									1

#define ISO_ENGLISH_CODE_STRING									"en"
#define ISO_ENGLISH_CODE_STRING_LENGTH					2
/**
  * @}
  */

/** @defgroup lib_NDEF_URI_ID
  * @{
  */
#define URI_ID_0x00								0x00
#define URI_ID_0x01								0x01
#define URI_ID_0x02								0x02
#define URI_ID_0x03								0x03
#define URI_ID_0x04								0x04
#define URI_ID_0x05								0x05
#define URI_ID_0x06								0x06
#define URI_ID_0x07								0x07
#define URI_ID_0x08								0x08
#define URI_ID_0x09								0x09
#define URI_ID_0x0A								0x0A
#define URI_ID_0x0B								0x0B
#define URI_ID_0x0C								0x0C
#define URI_ID_0x0D								0x0D
#define URI_ID_0x0E								0x0E
#define URI_ID_0x0F								0x0F
#define URI_ID_0x10								0x10
#define URI_ID_0x11								0x11
#define URI_ID_0x12								0x12
#define URI_ID_0x13								0x13
#define URI_ID_0x14								0x14
#define URI_ID_0x15								0x15
#define URI_ID_0x16								0x16
#define URI_ID_0x17								0x17
#define URI_ID_0x18								0x18
#define URI_ID_0x19								0x19
#define URI_ID_0x1A								0x1A
#define URI_ID_0x1B								0x1B
#define URI_ID_0x1C								0x1C
#define URI_ID_0x1D								0x1D
#define URI_ID_0x1E								0x1E
#define URI_ID_0x1F								0x1F
#define URI_ID_0x20								0x20
#define URI_ID_0x21								0x21
#define URI_ID_0x22								0x22
#define URI_ID_0x23								0x23
#define URI_RFU  									0x24
/**
  * @}
  */

/** @defgroup lib_NDEF_URI_ID_STRING
  * @{
  */
#define URI_ID_0x01_STRING								"http://www.\0"
#define URI_ID_0x02_STRING								"https://www.\0"
#define URI_ID_0x03_STRING								"http://\0"
#define URI_ID_0x04_STRING								"https://\0"
#define URI_ID_0x05_STRING								"tel:\0"
#define URI_ID_0x06_STRING								"mailto:\0"
#define URI_ID_0x07_STRING								"ftp://anonymous:anonymous@\0"
#define URI_ID_0x08_STRING								"ftp://ftp.\0"
#define URI_ID_0x09_STRING								"ftps://\0"
#define URI_ID_0x0A_STRING								"sftp://\0"
#define URI_ID_0x0B_STRING								"smb://\0"
#define URI_ID_0x0C_STRING								"nfs://\0"
#define URI_ID_0x0D_STRING								"ftp://\0"
#define URI_ID_0x0E_STRING								"dav://\0"
#define URI_ID_0x0F_STRING								"news:\0"
#define URI_ID_0x10_STRING								"telnet://\0"
#define URI_ID_0x11_STRING								"imap:\0"
#define URI_ID_0x12_STRING								"rtsp://\0"
#define URI_ID_0x13_STRING								"urn:\0"
#define URI_ID_0x14_STRING								"pop:\0"
#define URI_ID_0x15_STRING								"sip:\0"
#define URI_ID_0x16_STRING								"sips:\0"
#define URI_ID_0x17_STRING								"tftp:\0"
#define URI_ID_0x18_STRING								"btspp://\0"
#define URI_ID_0x19_STRING								"btl2cap://\0"
#define URI_ID_0x1A_STRING								"btgoep://\0"
#define URI_ID_0x1B_STRING								"tcpobex://\0"
#define URI_ID_0x1C_STRING								"irdaobex://\0"
#define URI_ID_0x1D_STRING								"file://\0"
#define URI_ID_0x1E_STRING								"urn:epc:id:\0"
#define URI_ID_0x1F_STRING								"urn:epc:tag\0"
#define URI_ID_0x20_STRING								"urn:epc:pat:\0"
#define URI_ID_0x21_STRING								"urn:epc:raw:\0"
#define URI_ID_0x22_STRING								"urn:epc:\0"
#define URI_ID_0x23_STRING								"urn:nfc:\0"
/**
  * @}
  */


uint16_t NDEF_IdentifyNDEF ( sRecordInfo *pRecordStruct, uint8_t* pNDEF );
uint16_t NDEF_ReadNDEF( uint8_t *pNDEF);
uint16_t NDEF_WriteNDEF( uint8_t *pNDEF);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif
#endif /* __LIB_NDEF_H */


/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
