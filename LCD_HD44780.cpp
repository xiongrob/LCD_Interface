

#include <string.h>
#include "LCD_HD44780.hh"
#include "Arduino.h"


/*
extern void pinMode( int, int );
extern void digitalWrite( int, int );
extern int digitalRead( int );
extern uint8_t digitalPinToBitMask( uint8_t );
extern uint8_t digitalPinToPort( uint8_t );
extern uint8_t* portModeRegister( uint8_t );
extern uint8_t* portOutputRegister( uint8_t );
void delay( int );

extern int INPUT;
extern int OUTPUT;
extern int HIGH;
extern int LOW;
extern int INPUT_PULLUP;
extern int NUM_DIGITAL_PINS;
*/

// nanoseconds
// static int en_min_pulse_width_c = 230;
static int rs_mask_c = 1 << 9;
static int rw_mask_c = 1 << 8;


static int get_pinMode(uint8_t pin)
    {
    if ( pin >= NUM_DIGITAL_PINS ) 
        return -1;
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    volatile uint8_t *reg = portModeRegister(port);
    if ( *reg & bit ) 
        return OUTPUT;

    volatile uint8_t *out = portOutputRegister(port);
    return (*out & bit) ? INPUT_PULLUP : INPUT;
    } // end get_pinMode( )


static bool check_db( const int8_t data_bus[ 8 ], const MPU_bit_interf bit_interface )
    {
    for ( int8_t const *db_ptr = data_bus + ( bit_interface == MPU_bit_interf::Four ? 4 : 0 ); 
        db_ptr != data_bus + dl_num_c; ++db_ptr )
        {
        if ( *db_ptr == -1 )
            return false;
        } // end for
    return true;
    } // end check_db( )

//---------------------------------------------------------------------------------------------------
//
//                                     LC_Display Definitions
//
//---------------------------------------------------------------------------------------------------


LCD_Display::LCD_Display( const int8_t _data_bus[ 8 ], const int _rs, const int _rw, const int _e, const MPU_bit_interf bit_inter )
        : rs( _rs ), rw( _rw ), en( _e ), v0( -1 ), interface_m( bit_inter ), MR_MPU( 0 ) 
    {
    if ( _data_bus )
        {
        set_data_bus( _data_bus );
        } // end if
    else
        {
        memset( data_bus, -1, sizeof( data_bus ) / sizeof( int8_t ) );
        } // end else

    pinMode( rs, OUTPUT );    
    pinMode( rw, OUTPUT );    
    pinMode( en, OUTPUT );    
    
    // Should be DAC pin?
    if ( v0 != -1 )
        pinMode( v0, OUTPUT );    

    } // end LCD_Display( )

void LCD_Display:: print_pins( ) const 
    {
    Serial.print( "RS: " );
    Serial.print( rs );
    Serial.print( "\n" );

    Serial.print( "RW: " );
    Serial.print( rw );
    Serial.print( "\n" );

    Serial.print( "EN: " );
    Serial.print( en );
    Serial.print( "\n" );


    Serial.print( "v0: " );
    Serial.print( v0 );
    Serial.print( "\n" );

    Serial.print( "Interface Mode: " );
    Serial.print( interface_m == MPU_bit_interf::Eight ? "8-bit" : "4-bit" );
    Serial.print( "\n" );

    Serial.print( "Data Bus: \n" );
    for ( int8_t n = 0; n < dl_num_c; ++n )
        {
        Serial.print( "DB" );
        Serial.print( n );
        Serial.print( ": " );
        Serial.print( data_bus[ n ] );
        Serial.print( "\n" );
        } // end for
    }
bool LCD_Display::attach_LCD_display( const int8_t _data_bus[ 8 ], const uint8_t _rs, const uint8_t _rw, const uint8_t _e, 
    const MPU_bit_interf bit_inter, const enum func_set n_f )
    {
    assert( _data_bus );
    Serial.print( "In attach_LCD_display\n" );

    set_data_bus( _data_bus );
    set_register_sel( _rs );
    set_read_write( _rw );
    set_enable( _e );

    print_pins( );
    disp_info = n_f;

    Serial.print( "Returning from attach_LCD_display\n" );
    return true;
    } // end attach_LCD_display( )

void LCD_Display::init_LCD( const Move_csr csr_mv, const Disp_Shft_on_w shift_disp, const bool csr_on, const bool csr_blinks )
    {
    init_LCD( interface_m, disp_info, mv_crsr, shfts_w, csr_on, csr_blinks );
    } // end init_LCD( )

// Initiailzation By Instructions B/c necessary if the power supply conditions for correctly operating
// the internal reste aren't met.
void LCD_Display::init_LCD( const MPU_bit_interf bit_inter, const enum func_set n_f, const Move_csr csr_mv, const Disp_Shft_on_w shift_disp, 
                            const bool csr_on, const bool csr_blinks )
    {
    static bool has_been_called = false;
    if ( has_been_called )
        return;
    
    has_been_called = true; // Ensures only is done once.

    assert( is_good( ) );
    // 1) Wait for more than 15 ms after Vcc rises to 4.5 V (or 40 ms after Vcc rises to 2.7 V)
    delay( 40 );

    //! Donot check BF before this instruction.
    // Function set(8-bits)
    interface_m = MPU_bit_interf::Eight; // Needs to act as 8-bit interface for a bit.
    function_set( MPU_bit_interf::Eight, func_set::one_line_5x10, false );

    // 3) Wait for more than 4.1 ms
    delay( 5 ); 
    function_set( MPU_bit_interf::Eight, func_set::one_line_5x10, false );

    // Wait for more than 100us
    delay( 1 );
    function_set( MPU_bit_interf::Eight, func_set::one_line_5x10, false );

    // Now BF can be checked for following instructions
    interface_m = bit_inter; // Change back to original

    // Function set (specify number of display lines and character font).
    //! Number of display lines and character font cannot be changed after this point
    function_set( bit_inter, n_f );

    // Display Off
    display_state_control( false, false, false );

    // Display clear
    clear_display( );

    // Entry mode set
    entry_m_set( csr_mv, shift_disp );

    // Set Display According to desires
    display_state_control( true, csr_on, csr_blinks );

    // Done
    } // end init_LCD( )

void LCD_Display::set_data_bus( const int8_t data_bus_pins [ 8 ] )
    {  
    for ( int8_t *db_ptr = data_bus; 
        db_ptr != data_bus + sizeof( data_bus ) / sizeof( int8_t ); ++db_ptr, ++data_bus_pins )
        {
        if ( *data_bus_pins != -1 )
            *db_ptr = *data_bus_pins;
        } // end for
    } // end set_data_bus( )

void LCD_Display::set_register_sel( const uint8_t rs_pin )
    { 
    rs = rs_pin; 
    pinMode( rs, OUTPUT );
    } // end set_register_sel( )

void LCD_Display::set_read_write( const uint8_t rw_pin )
    { 
    rw = rw_pin; 
    pinMode( rw, OUTPUT );
    } // end set_read_write( )
bool LCD_Display::is_good( ) const
    {
    return     check_db( data_bus, interface_m )
            && rs != -1
            && get_pinMode( rs ) == OUTPUT
            && rw != -1
            && get_pinMode( rw ) == OUTPUT
            && en != -1;
    } // end is_good( )


void LCD_Display::set_enable( const uint8_t e_pin )
    { 
    en = e_pin; 
    pinMode( en, OUTPUT );
    } // end set_enable( )


void LCD_Display::set_bit_interface( const MPU_bit_interf bit_inter )
    { 
    interface_m = bit_inter; 
    } // end set_bit_interface( )

bool LCD_Display::clear_display( )
    {
    if ( !is_good( ) )
        return false;
    uint16_t inst = HD44790U_Inst::clear_display;
    mpu_interface( inst );
    return true;
    } // end clear_display( )

bool LCD_Display::ret_home( )
    {
    if ( !is_good( ) )
        return false;
    uint16_t inst = HD44790U_Inst::ret_home;
    mpu_interface( inst );
    return true;
    } // end ret_home( )

void LCD_Display::entry_m_set( const Move_csr csr_mv, const Disp_Shft_on_w shift_disp )
    {
    mv_crsr = csr_mv;       
    shfts_w = shift_disp;
    uint16_t inst = HD44790U_Inst::entry_m_set;
    inst |= static_cast< int >( csr_mv ) << 1;
    inst |= static_cast< int >( shift_disp );
    mpu_interface( inst );
    } // end ret_home( )

void LCD_Display::toggle_display( const bool on )
    {
    assert ( false );
    } // end toggle_display( )

void LCD_Display::toggle_cursor( const bool on )
    {
    assert ( false );
    } // end toggle_cursor( )

void LCD_Display::toggle_cursor_blink( const bool on )
    {
    assert ( false );
    } // end toggle_cursor_blink( )

void LCD_Display::display_state_control( const bool disp_on, const bool csr_on, const bool csr_blinks )
    {
    Serial.print( "In display_state_control\n" );
    uint16_t inst = HD44790U_Inst::disp_io_ctrl;
    inst |= ( disp_on ? 1 : 0 ) << 2;
    inst |= ( csr_on ? 1 : 0 ) << 1;
    inst |= ( csr_blinks ? 1 : 0 );
    mpu_interface( inst );
    } // end display_state_control( )

uint16_t shift_helper( const D_C d_or_c,  const Direction dir );
void LCD_Display::move_cursor( const Direction dir )
    {
    uint16_t inst = shift_helper( D_C::Cursor, dir ); 
    mpu_interface( inst );
    } // end move_cursor( )

void LCD_Display::shift_display( const Direction dir )
    {
    uint16_t inst = shift_helper( D_C::Display , dir ); 
    mpu_interface( inst );
    } // end shift_display( )

uint16_t shift_helper( const D_C d_or_c,  const Direction dir )
    {
    uint16_t inst = HD44790U_Inst::csr_disp_shft;
    inst |= static_cast< int > ( d_or_c ) << 3; // S/C
    inst |= static_cast< int >( dir ) << 2; // D/L
    return inst;
    } // end shift_helper( )

void LCD_Display::function_set( const MPU_bit_interf bit_interface, const enum func_set n_f, const bool check_BF )
    {
    Serial.print( "In function_set\n" );
    interface_m = bit_interface;
    disp_info = n_f;
    uint16_t inst = HD44790U_Inst::func_set;
    inst |= static_cast< int > ( bit_interface ) << 4; // S/C
    inst |= static_cast< int > ( n_f ) << 2; // S/C
    mpu_interface( inst, check_BF );
    } // end function_set( 0)

void LCD_Display::set_CGRAM_addr( const uint8_t addr_counter )
    {
    uint16_t inst = HD44790U_Inst::set_cgram_addr;
    inst |= addr_counter; // S/C
    mpu_interface( inst );
    } // end set_CGRAM_addr( )

void LCD_Display::set_DDRAM_addr( const uint8_t addr_counter )
    {
    uint16_t inst = HD44790U_Inst::set_ddram_addr;
    inst |= addr_counter; // S/C
    mpu_interface( inst );
    } // end set_DDRAM_addr( )

bool LCD_Display::is_busy( ) const
    {
    return read( ) & DB7;
    } // end is_busy( )

uint8_t LCD_Display::get_addr_counter( ) const
    {
    return read( ) & ( ( 1 << 7 ) - 1 );
    } // end get_addr_counter( )

void LCD_Display::write_CG_DDRAM( const uint8_t data_byte )
    {
    uint16_t inst = HD44790U_Inst::wr_data_CG_DDRAM;
    inst |= data_byte;
    mpu_interface( inst );
    } // end write_CG_DDRAM( )


uint8_t LCD_Display::read_CG_DDRAM( )
    {
    uint16_t inst = HD44790U_Inst::rd_data_CG_DDRAM;
    mpu_interface( inst );
    return static_cast< uint8_t >( inst );
    } // end read_CG_DDRAM( )

void LCD_Display::mpu_interface( uint16_t& bits, const bool check_BF )
    {
    Serial.print( "Call mpu_interface: ");
    Serial.print( bits, BIN );
    Serial.print( "\n" );

    // assert( is_good( ) );
    const bool is_read = bits & rw_mask_c; // True if read, false if write

    
    while( check_BF && is_busy( ) ) // Wait until BF==0
        {
        Serial.print( "is_busy\n" );
        }

    if ( is_read )
        bits |= read( bits & rs_mask_c );
    else
        write( static_cast< uint8_t >( bits ), bits & rs_mask_c ); // Chop the lower 8 bits.
    
    MR_MPU = bits;
    } // end send_instr( )

void write_four_db( const int8_t data_bus[ 4 ], const uint8_t data );
void LCD_Display::write( const uint8_t db0_7, const bool data )
    {
    set_db_io( OUTPUT ); // Set pin mode appropiately
    digitalWrite( rs, data ? HIGH : LOW ); // Send whether is read or write
    digitalWrite( rw, LOW ); // To signify write
    digitalWrite( en, HIGH ); // Pull up enable

    Serial.print( "Sending for DB7 - DB4: 0b" );
    Serial.print( db0_7 >> 4, BIN );
    Serial.print( "\n" );
    write_four_db( data_bus + 4, db0_7 >> 4 ); // Send db4 - db7
    if ( interface_m == MPU_bit_interf::Four )
        {
        // Pull down enable and up again
        // Setup (tDSW == 80ns )
        delay( 1 );
        digitalWrite( en, LOW ); // Pull down enable
        // Hold time (hH == 10ns ) -> Way too low to be of concern
        digitalWrite( en, HIGH ); // Pull up enable
        write_four_db( data_bus + 4, db0_7 ); // Send db0 - db3
        } // end if
    else
        {
        write_four_db( data_bus, db0_7 ); // Send db0 - db3
        } // end else
    Serial.print( "Sending for DB0 - DB3: 0b" );
    Serial.print( db0_7 & ( ( 1 << 4 ) - 1 ), BIN );
    Serial.print( "\n" );
    delay( 1 );

    digitalWrite( en, LOW ); // Pull down enable
    } // end write_inst( )

void write_four_db( const int8_t data_bus[ 4 ], const uint8_t data )
    {
    int n = 1;
    for ( int8_t const *db_ptr = data_bus; db_ptr != data_bus + 4; ++db_ptr, n <<= 1 )
        {
        digitalWrite( *db_ptr, data & n );
        } // end for
    } // end write_four_db( )

uint8_t read_four_db( const int8_t data_bus[ 4 ] );
uint8_t LCD_Display::read( const bool data ) const
    {
    set_db_io( INPUT ); // Set pin mode to in to read
    digitalWrite( rs, data ? HIGH : LOW ); // Send whether is read or write
    digitalWrite( rw, HIGH ); // To signify read
    digitalWrite( en, HIGH ); // Pull up enable
    uint8_t db;
    int8_t const *db_ptr = data_bus + 4;

    db = read_four_db( db_ptr ); // Read in DB4 - DB7

    Serial.print( "For DB4 - DB7 got: 0b" );
    Serial.print( db, BIN );
    Serial.print( "\n" );
    if ( interface_m == MPU_bit_interf::Four )
        {
        // Pull down enable and up again
        digitalWrite( en, LOW ); // Pull down enable
        digitalWrite( en, HIGH ); // Pull up enable
        } // end if
    else
        {
        db_ptr = data_bus; // To the read the bottom 4 wires in 8-bit mode.
        } // end else
    db <<= 4;
    db |= read_four_db( data_bus ); // Read in DB0 - DB3

    Serial.print( "For DB3 - DB0 got: 0b" );
    Serial.print( db >> 4, BIN );
    Serial.print( "\n" );

    digitalWrite( en, LOW ); // Pull down enable
    return db;
    } // end write_inst( )

uint8_t read_four_db( const int8_t data_bus[ 4 ] )
    {
    uint8_t nibble = 0;
    for ( int8_t const *db_ptr = data_bus; db_ptr != data_bus + 4; ++db_ptr )
        {
        nibble <<= 1;
        nibble |= digitalRead( *db_ptr );
        } // end for
    return nibble;
    } // end read_four_db( )


// Sets the data bus pins to certain direction (depending on instruction)
// And ignores the lower 4 data bus pins (if in 4-bit mode).
void LCD_Display::set_db_io( const int pin_mode ) const
    {
    for ( int8_t const *db_ptr = data_bus + ( interface_m == MPU_bit_interf::Four ? 4 : 0 ); 
        db_ptr != data_bus + sizeof( data_bus ) / sizeof( int8_t ); ++db_ptr )
        {
        pinMode( *db_ptr, pin_mode );
        } // end for
    } // end set_db_io( )
