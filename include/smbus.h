#ifndef _SMBUS_H_
#define _SMBUS_H_

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/

#include <stdint.h>


/*****************************************************************************/
/* Defines                                                                   */
/*****************************************************************************/

#define SMBUS_STATUS_ERROR                  ( -1 )
#define SMBUS_STATUS_OK                     ( 0 )

#define SMBUS_MAX_DATA                      ( 255 )
#define SMBUS_MAX_BUFFER                    ( SMBUS_MAX_DATA + 3 ) /* cmd + len + pec */

#define SMBUS_MIN_BITRATE_KHZ               ( 100 )
#define SMBUS_MAX_BITRATE_KHZ               ( 1000 )

#define SMBUS_TARGET_NO_WAIT                ( 0 )
#define SMBUS_TARGET_WAIT_FOREVER           ( -1 )

#define INJECT_INVALID_PEC( c )            smbus_inject_invalid_pec( c );
#define INJECT_INVALID_BLOCK_SIZE( c )     smbus_inject_invalid_block_size( c );

/*****************************************************************************/
/* Structs                                                                   */
/*****************************************************************************/

/**
 * @struct SMBUS_TARGET_PEC_INFO
 * @brief  Info about PEC when setting up a data response as a target.
 * 
 * @note   If `bPreferPrecalculatedPEC` is TRUE preference will be given
 *         to calculating the PEC byte when setting the response as opposed
 *         to calculating it on the fly. This does not guarantee any behaviour -
 *         the exact calculation method depends on the underlying implementation
 *         and what it is capable of. If passing the value as TRUE, the expected
 *         data buffer and length must also be passed in.
 */
typedef struct SMBUS_TARGET_PEC_INFO
{
    bool     bPecEnabled;
    bool     bPreferPrecalculatedPEC;
    uint8_t  pucExpectedData[ SMBUS_MAX_DATA ];
    uint16_t usExpectedDataLen;

} SMBUS_TARGET_PEC_INFO;

/*****************************************************************************/
/* Enums                                                                     */
/*****************************************************************************/

/**
 * @enum  SMBUS_TARGET_GET_DATA_FILTER
 * @brief Data event filters when receiving data as a target.
 */
typedef enum SMBUS_TARGET_GET_DATA_FILTER
{
    SMBUS_TARGET_GET_DATA_ANY           = 0x01, /* Any data events, including no data. */
    SMBUS_TARGET_GET_DATA_INCOMING      = 0x02, /* Successful data read events. */
    SMBUS_TARGET_GET_DATA_OUTGOING      = 0x04, /* Successful data write events. */
    SMBUS_TARGET_GET_DATA_IGNORE_ERRORS = 0x08  /* Ignores errors when used in combination with previous filters. */

} SMBUS_TARGET_GET_DATA_FILTER;

/**
 * @enum  SMBUS_INJECT_BLOCK_SIZE_ERROR
 * @brief enum defining block size error to be injected
 */
typedef enum SMBUS_INJECT_BLOCK_SIZE_ERROR
{
    SMBUS_INJECT_BLOCK_SIZE_NO_ERROR    = 0x00, /* no block size error to be injected. */
    SMBUS_INJECT_BLOCK_SIZE_SMALL_ERROR = 0x01, /* inject smaller than expected block size. */
    SMBUS_INJECT_BLOCK_SIZE_LARGE_ERROR = 0x02, /* inject larger than expected block size. */

} SMBUS_INJECT_BLOCK_SIZE_ERROR;

/*****************************************************************************/
/* Public function declarations - General                                    */
/*****************************************************************************/

/*****************************************************************************/
/*
 * @brief Initialise SMBus device
 *
 * @param   iFile               Device file to open
 * @param   iBitRateKHz         Bus bit rate (in KHz)
 *
 * @return  SMBUS_STATUS_OK     SMBus device opened
 *          SMBUS_STATUS_ERROR  SMBus device not opened
 */
/*****************************************************************************/
extern int smbus_init( int iFile,
                       int iBitRateKHz );

/*****************************************************************************/
/*
 * @brief De-Initialise SMBus device
 *
 * @param   None
 *
 * @return  SMBUS_STATUS_OK     SMBus device closed
 *          SMBUS_STATUS_ERROR  SMBus device not closed
 */
/*****************************************************************************/
extern int smbus_deinit( void );

/*****************************************************************************/
/*
 * @brief   This function updates bInvalidPec variable in static 
 *          SMBUS_TEST_CTRL struct
 *
 * @param   bInvalidPec         Invalid PEC bool flag
 *
 * @return  N/A
 */
/*****************************************************************************/
extern void smbus_inject_invalid_pec( bool bInvalidPec );

/*****************************************************************************/
/*
 * @brief   This function updates xInvalidBlockSize variable in static 
 *          SMBUS_TEST_CTRL struct
 *
 * @param   xInvalidBlockSize    Invalid Block Size enum
 *
 * @return  N/A
 */
/*****************************************************************************/
extern void smbus_inject_invalid_block_size( SMBUS_INJECT_BLOCK_SIZE_ERROR xInvalidBlockSize );

/*****************************************************************************/
/* Public function declarations - Controller                                 */
/*****************************************************************************/

/*****************************************************************************/
/*
 * @brief Call SMBus command "Send byte"
 *
 * @param   ucAddr              Remote target address
 * @param   ucData              Data to write
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_send_byte( uint8_t ucAddr,
                            uint8_t ucData,
                            bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Receive byte"
 *
 * @param   ucAddr              Remote target address
 * @param   ucData              Data read
 * @param   bPec                TRUE    - expect to receive a PEC byte
 *                              FALSE   - do not expect a PEC byte
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_receive_byte( uint8_t ucAddr,
                               uint8_t* pucData,
                               bool bPec );


/*****************************************************************************/
/*
 * @brief Call SMBus command "Write byte"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   ucData              Data to write
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_write_byte( uint8_t ucAddr,
                             uint8_t ucCmd,
                             uint8_t ucData,
                             bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Read byte"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   pucData             Data read
 * @param   bPec                TRUE    - expect to receive a PEC byte
 *                              FALSE   - do not expect a PEC byte
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_read_byte( uint8_t ucAddr,
                            uint8_t ucCmd,
                            uint8_t* pucData,
                            bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Write word"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   usData              Data to write
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_write_word( uint8_t ucAddr,
                             uint8_t ucCmd,
                             uint16_t usData,
                             bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Read Word"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   pusData             Data read
 * @param   bPec                TRUE    - expect to receive a PEC byte
 *                              FALSE   - do not expect a PEC byte
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_read_word( uint8_t ucAddr,
                            uint8_t ucCmd,
                            uint16_t* pusData,
                            bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Write 32"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   ulData              Data to write
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_write_32( uint8_t ucAddr,
                           uint8_t ucCmd,
                           uint32_t ulData,
                           bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Read 32"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   pulData             Data read
 * @param   bPec                TRUE    - expect to receive a PEC byte
 *                              FALSE   - do not expect a PEC byte
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_read_32( uint8_t ucAddr,
                          uint8_t ucCmd,
                          uint32_t* pulData,
                          bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Write 64"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   ullData             Data to write
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_write_64( uint8_t ucAddr,
                           uint8_t ucCmd,
                           uint64_t ullData,
                           bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Read 64"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   *pullData           Data read
 * @param   bPec                TRUE    - expect to receive a PEC byte
 *                              FALSE   - do not expect a PEC byte
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_read_64( uint8_t ucAddr,
                          uint8_t ucCmd,
                          uint64_t* pullData,
                          bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Block write"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   pucData             Data buffer to write
 * @param   ucLength            Number of bytes to write
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_block_write( uint8_t ucAddr,
                              uint8_t ucCmd,
                              uint8_t* pucData,
                              uint8_t ucLength,
                              bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Block read"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   pucData             Data buffer read
 * @param   pucLength           Number of bytes read
 * @param   bPec                TRUE    - expect to receive a PEC byte
 *                              FALSE   - do not expect a PEC byte
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_block_read( uint8_t ucAddr,
                             uint8_t ucCmd,
                             uint8_t* pucData,
                             uint8_t* pucLength,
                             bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Process call"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   usWrData            Data to write
 * @param   pusRdData           Data read
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_process_call( uint8_t ucAddr,
                               uint8_t ucCmd,
                               uint16_t usWrData,
                               uint16_t* pusRdData,
                               bool bPec );

/*****************************************************************************/
/*
 * @brief Call SMBus command "Block Write/Read Process call"
 *
 * @param   ucAddr              Remote target address
 * @param   ucCmd               Protocol command ID
 * @param   pucWrData           Data buffer to write
 * @param   ucWrLen             Number of bytes to write
 * @param   pusRdData           Data read
 * @param   pucRdLen            Number of bytes read
 * @param   bPec                TRUE    - include PEC
 *                              FALSE   - do not include PEC
 *
 * @return  SMBUS_STATUS_OK     SMBus operation successful
 *          SMBUS_STATUS_ERROR  SMBus operation unsuccessful
 */
/*****************************************************************************/
extern int smbus_block_process_call( uint8_t ucAddr,
                                     uint8_t ucCmd,
                                     uint8_t* pucWrData,
                                     uint8_t ucWrLen,
                                     uint8_t* pucRdData,
                                     uint8_t* pucRdLen,
                                     bool bPec );


/*****************************************************************************/
/* Public function declarations - Target                                     */
/*****************************************************************************/

/*****************************************************************************/
/*
 * @brief Enable an SMBus target at a fixed address
 *
 * @param   ucAddr              Address of target
 *
 * @return  SMBUS_STATUS_OK     Target enabled and ready to receive
 *          SMBUS_STATUS_ERROR  Target not enabled
 *
 * @notes   Only 1 target address can be active at any time.
 */
/*****************************************************************************/
extern int smbus_target_enable( uint8_t ucAddr );

/*****************************************************************************/
/*
 * @brief Disable the SMBus target
 *
 * @param   None
 *
 * @return  SMBUS_STATUS_OK     Target disabled
 *          SMBUS_STATUS_ERROR  Target not disabled
 */
/*****************************************************************************/
extern int smbus_target_disable( void );

/*****************************************************************************/
/*
 * @brief Prepare data for target transmission
 *
 * @param   pucData             Data buffer for sending
 * @param   ucLength            Max length to send
 * @param   pxPecInfo           Information relevant to calculating PEC
 *
 * @return  SMBUS_STATUS_OK     Data loaded
 *          SMBUS_STATUS_ERROR  Data not loaded
 *
 * @notes   If PEC is enabled, ucLength will be incremented within the function.
 *          The normal max value of ucLength is 255, so if PEC is required,
 *          the maximum value of ucLength is 254.
 *
 *          -----------------------------------------------------------------
 *          - Totalphase Aardvark specific:
 *          For the TotalPhase Aardvark, the maximum buffer size is 64. Any
 *          responses greater than 64 bytes will be wrapped around.
 *          Therefore, for responses that require PEC, the maximum value of
 *          ucLength should be limited to 63.
 *          -----------------------------------------------------------------
 */
/*****************************************************************************/
extern int smbus_target_set_data( uint8_t* pucData,
                                  uint8_t ucLength,
                                  SMBUS_TARGET_PEC_INFO* pxPecInfo );

/*****************************************************************************/
/*
 * @brief Retrieve data sent to target
 *
 * @param   pucData             Buffer for data received
 * @param   pusLength           Size of the data buffer
 *                              - Updated to the actual size of data received
 * @param   eDataFilter         Data event filter
 * @param   iTimeoutMs          Duration (in ms) to wait for data
 * @param   bPecExpected        Will the controller send a PEC byte
 *
 * @return  SMBUS_STATUS_OK     Correct data received within specified timeout
 *                              based on the data level set by `eDataFilter`
 *          SMBUS_STATUS_ERROR  No valid data received
 *
 *  @notes  To exit immediately without waiting:
 *              set iTimeoutMs to SMBUS_TARGET_NO_WAIT
 *          To wait indefinitely for data:
 *              set iTimeoutMs to SMBUS_TARGET_WAIT_FOREVER
 *          
 *          -----------------------------------------------------------------
 *          If a "no data" event is received, pucData is cleared and pusLength
 *          set to 0. If a "write data" event is received pusLength is updated
 *          to the number of bytes written. In the case of a "read data" event,
 *          pucData is updated appropriately to the received data, only if
 *          the received data size is > 0 and less than the value of pusLength
 *          passed into the function; if the return value is SMBUS_STATUS_OK,
 *          pusLength will also have been updated even if the received data
 *          was invalid or there was an error.
 *          -----------------------------------------------------------------
 */
/*****************************************************************************/
extern int smbus_target_get_data( uint8_t* pucData,
                                  uint16_t* pusLength,
                                  SMBUS_TARGET_GET_DATA_FILTER eDataFilter,
                                  int iTimeoutMs,
                                  bool bPecExpected );

#endif //_SMBUS_H_
