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

void Assert( const bool statement, char const *c_string )
    {
    if ( !statement )
        {  
        for ( ; *c_string; ++c_string )
            Serial.print( *c_string );
        
        delay( 1000 * strlen( c_string ) ); // Give enough time for characters to be pushed through serial port.
        assert( false );
        } // end if
    } // end Assert( )

void Csr_Pos_t::print( ) const
    {
    Serial.print( "Row: " );
    Serial.print( row );
    Serial.print( "\n" );
    Serial.print( "Pos: " );
    Serial.print( pos );    
    Serial.print( "\n" );
    Serial.print( "Abs Pos: " );
    Serial.print( abs_pos ? "True" : "False" );    
    Serial.print( "\n" );
    } // end print( )
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
    } // end pin_pins( )
bool LCD_Display::attach_LCD_display( const int8_t _data_bus[ 8 ], const uint8_t _rs, const uint8_t _rw, const uint8_t _e, 
    const MPU_bit_interf bit_inter, const enum func_set n_f )
    {
    assert( _data_bus );
    Serial.print( "In attach_LCD_display\n" );

    set_data_bus( _data_bus );
    set_register_sel( _rs );
    set_read_write( _rw );
    set_enable( _e );
    set_bit_interface( bit_inter );

    print_pins( );
    disp_info = n_f;

    Serial.print( "Returning from attach_LCD_display\n" );
    return true;
    } // end attach_LCD_display()

void LCD_Display::init_LCD( const Csr_Dir csr_mv, const Disp_Shft_on_w shift_disp, const bool csr_on, const bool csr_blinks )
    {
    init_LCD( interface_m, disp_info, csr_mv, shift_disp, csr_on, csr_blinks );
    } // end init_LCD( )

// Initiailzation By Instructions B/c necessary if the power supply conditions for correctly operating
// the internal reste aren't met.
void LCD_Display::init_LCD( const MPU_bit_interf bit_inter, const enum func_set n_f, const Csr_Dir csr_mv, const Disp_Shft_on_w shift_disp, 
                            const bool csr_on, const bool csr_blinks )
    {
    static bool has_been_called = false;
    if ( has_been_called )
        return;
    
    has_been_called = true; // Ensures only is done once.

    interface_m = bit_inter; // Needs to be done before is_good is called.
    Assert( is_good( ), "Something is wrong with the pins(From is_good( ))\n" );

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
    delayMicroseconds( 101 );
    function_set( MPU_bit_interf::Eight, func_set::one_line_5x10, false );

    // Now BF can be checked for following instructions

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
    d_c_b = DB2 | csr_on << 1 | csr_blinks;
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
    Assert( get_pinMode( rs ) == OUTPUT, "RS isn't OUTPUT\n" );
    } // end set_register_sel( )

void LCD_Display::set_read_write( const uint8_t rw_pin )
    { 
    rw = rw_pin; 
    pinMode( rw, OUTPUT );
    Assert( get_pinMode( rw ) == OUTPUT, "RW isn't OUTPUT\n" );
    } // end set_read_write( )

void LCD_Display::set_enable( const uint8_t e_pin )
    { 
    en = e_pin; 
    pinMode( en, OUTPUT );
    Assert( get_pinMode( en ) == OUTPUT, "EN isn't OUTPUT\n" );
    } // end set_enable( )


void LCD_Display::set_bit_interface( const MPU_bit_interf bit_inter )
    { 
    interface_m = bit_inter; 
    } // end set_bit_interface( )

bool LCD_Display::is_good( ) const
    {
    return     check_db( data_bus, interface_m )
            && rs != -1
            && get_pinMode( rs ) == OUTPUT
            && rw != -1
            && get_pinMode( rw ) == OUTPUT
            && en != -1
            && get_pinMode( en ) == OUTPUT;
    } // end is_good( )

//---------------------------------------------------------------------------------------------------
//
//                                     Top-Level Interface Declarations
//
//---------------------------------------------------------------------------------------------------    

bool LCD_Display::type_chars( const char *c_string, const Csr_Pos_t csr_pos_strt )
    {
    if ( csr_pos_strt != Csr_Pos_t( -1, -1, false ) )
        {
        if ( !csr_pos_strt.abs_pos )
            csr_pos_strt = rel_to_abs_pos( csr_pos_strt );

        if ( csr_pos_strt.valid( get_display_upper_pos( ), disp_info & row_bit_c ) )
            if ( !jmp_abs_pos( csr_pos_strt, false ) )
                return false;
        } // end if
    for ( char const *ptr = c_string; *ptr; ++ptr )
        {
        put_char( *ptr );
        } // end for
    return true;
    } // end type_chars( )

void LCD_Display::put_char( const char character )
    {
    Assert( csr_dir == Csr_Dir::Increment, "Decrement not yet supported\n" );
    // Check bounds and move cursor if necessary
    if ( csr_bounds == Cursor_Bounds::Ln_Br 
        && abs_to_rel_pos( csr_pos ).pos >= upper_rel_pos_c )
        {
        // Force the cursor to go to the new line.
        if ( disp_info & row_bit_c )
            {
            goto_next_ln( );
            }
        // Guess we keep going else.
        } // end if
    // Else keep going.
    // Update cursor position
    write_CG_DDRAM( character, can_chk_bf( ) );
    update_csr_pos( Direction::Right );
    } // end put_char( )

bool LCD_Display::jmp_abs_pos( const Csr_Pos_t new_pos, const bool check_BF )
    {
    Assert( new_pos.abs_pos, "new_pos is not absolute position" );
    if ( !csr_pos.valid( get_display_upper_pos( ), disp_info & row_bit_c ) )
        {
        Serial.print( "new_pos isn't valid\n" );
        new_pos.print( );
        return false;
        } // end i

    csr_pos = new_pos;

    // Ensures a range of [0, 40] for row one
    // and [0x40 to 0x67] for row two.
    set_DDRAM_addr( csr_to_DDRAM_addr( new_pos ), check_BF ); 
    return true;
    } // end jmp_abs_pos( )

void LCD_Display::goto_next_ln( )
    {
    if ( disp_info & row_bit_c )
        Assert( jmp_abs_pos(Csr_Pos_t( !csr_pos.row, disp_strt_pos, true ), false ), 
            "goto_next_ln failed\n" );
    } // end got_to_next_ln( 0)

void LCD_Display::clear_row( const int8_t row_num )
    {
    // Checks if row num is gte to 0 and lt 1 or 2 (depending of display info).
    assert( row_num >= 0 && row_num < ( disp_info & row_bit_c ) + 1 );

    Csr_Pos_t hold_pos = csr_pos;
    // Move cursor to beginning.

    jmp_abs_pos( { row_num, 0, true }, false );

    if ( shfts_w == Disp_Shft_on_w::Yes )
        entry_m_set( csr_dir, Disp_Shft_on_w::No ); // Temporarily

    // Write spaces to all 40 digits
    for ( int n = 0; n < get_display_upper_pos( ); ++n )
        write_CG_DDRAM( ' ', false ); //! Needs to override BF check 
    Assert( csr_pos == Csr_Pos_t( row_num, 0, true ), "Cursor should be back at pos 0" );
    // Restore previous cursor position.
    csr_pos = hold_pos;

    if ( shfts_w == Disp_Shft_on_w::Yes )
        entry_m_set( csr_dir, Disp_Shft_on_w::Yes ); // Change back.
        
    jmp_abs_pos( csr_pos ); // Ensure that the entry set is properly done.
    } // end clear_row( )

void LCD_Display::shift_cursor ( const Direction dir )
    {
    if ( dir == Direction::Up )
        {
        if ( csr_pos.row > 0 )
            {
            --csr_pos.row;
            if ( !jmp_abs_pos( csr_pos ) )
                {
                Serial.print( "Failed jmp_abs_pos in shift_cursor Direction:Up\n" );
                Serial.print( "Row: " ); Serial.print( csr_pos.row ); Serial.print( "\n" );
                Serial.print( "Pos: " ); Serial.print( csr_pos.pos ); Serial.print( "\n" );
                assert ( false );
                } // end if
            } // end if
        } // end if
    else if ( dir == Direction::Down )
        {
        if ( csr_pos.row >= 0 && csr_pos.row < ( disp_info & row_bit_c ) + 1 )
            {
            ++csr_pos.row;
            if ( !jmp_abs_pos( csr_pos ) )
                {
                Serial.print( "Failed jmp_abs_pos in shift_cursor Direction:Down\n" );
                Serial.print( "Row: " ); Serial.print( csr_pos.row ); Serial.print( "\n" );
                Serial.print( "Pos: " ); Serial.print( csr_pos.pos ); Serial.print( "\n" );
                assert ( false );
                } // end if
            } // end if
        } // end else if
    else if ( dir == Direction::Right )
        {
        ++csr_pos.pos;
        move_cursor( Direction::Right );
        } // end else if
    else // Direction::Left
        {
        --csr_pos.pos;
        move_cursor( Direction::Left );
        } // end else
    } // end shift_cursor( )

void LCD_Display::display_ctrl( const bool on )
    {
    d_c_b &= ( on << 2 | ~DB2 );
    display_state_control( on, d_c_b & DB1, d_c_b & DB0 );
    } // end display_ctrl( )

void LCD_Display::csr_ctrl( const bool csr_on, const bool blink )
    {
    d_c_b &= ( DB2 | csr_on << 1 | blink );
    display_state_control( d_c_b & DB2, csr_on, blink );
    } // end csr_ctrl( )

void LCD_Display::toggle_display( )
    {
    d_c_b ^= DB2;
    display_state_control( d_c_b & DB2, d_c_b & DB1, d_c_b & DB0 );
    } // end toggle_display( )

void LCD_Display::toggle_cursor( )
    {
    d_c_b ^= DB1;
    display_state_control( d_c_b & DB2, d_c_b & DB1, d_c_b & DB0 );
    } // end toggle_cursor( )

void LCD_Display::toggle_cursor_blink( )
    {
    d_c_b ^= DB0;
    display_state_control( d_c_b & DB2, d_c_b & DB1, d_c_b & DB0 );
    } // end toggle_cursor_blink( )

Csr_Pos_t LCD_Display::rel_to_abs_pos ( const Csr_Pos_t rel_pos ) const
        {
        Assert( !rel_pos.abs_pos && rel_pos.valid( upper_rel_pos_c, disp_info & row_bit_c ), 
            "Needs to be relative position and/or pos not 0 <= x <= 16\n" );

        Csr_Pos_t abs_pos( rel_pos.row, 
                           (disp_strt_pos + rel_pos.pos) % get_display_upper_pos( ), 
                           true );
        return abs_pos;
        } // end rel_to_abs_pos( )

Csr_Pos_t LCD_Display::abs_to_rel_pos ( const Csr_Pos_t abs_pos ) const
        {
        Assert( abs_pos.abs_pos && abs_pos.valid( get_display_upper_pos( ), disp_info & row_bit_c ), 
            "Needs to be absolute position and/or pos not 0 <= x <= 40\n" );

        Csr_Pos_t rel_pos( abs_pos.row, 
                           ( abs_pos.pos - disp_strt_pos ), 
                           false );
        if ( disp_strt_pos > abs_pos.pos )
            rel_pos.pos + get_display_upper_pos( );
        return rel_pos;
        } // end rel_to_abs_pos( )

void LCD_Display::update_disp_strt_pos( const Direction disp_mv_dir )
    {
    disp_strt_pos = update_pos_helper( disp_strt_pos, disp_mv_dir, Direction::Left );
    } // end update_disp_strt_pos( )

void LCD_Display::update_csr_pos( const Direction disp_mv_dir )
    {
    csr_pos.pos = update_pos_helper( csr_pos.pos, disp_mv_dir, Direction::Right );
    } // end update_csr_pos( )

int8_t LCD_Display::update_pos_helper( int8_t curr_pos, const Direction mv_dir, const Direction mv_right ) const
    {
    assert( mv_dir == Direction::Left || mv_dir == Direction::Right );
    curr_pos += ( mv_dir == mv_right ? 1 : get_display_upper_pos( ) - 1 );
    curr_pos %= get_display_upper_pos( );
    return curr_pos;
    } // end update_pos_helper( )

//---------------------------------------------------------------------------------------------------
//
//                                     MPU Interface Definitions
//
//---------------------------------------------------------------------------------------------------    

bool LCD_Display::clear_display( const bool check_BF )
    {
    if ( !is_good( ) )
        return false;
    uint16_t inst = HD44790U_Inst::clear_display;
    mpu_interface( inst, check_BF );

    // Return the cursor position to {0,0}
    csr_pos = { 0, 0, true };
    disp_strt_pos = 0;
    delay( 10 ); // Wait about 10 milliseconds.
    return true;
    } // end clear_display( )

bool LCD_Display::ret_home( const bool check_BF )
    {
    if ( !is_good( ) )
        return false;
    Serial.print( "In ret_home\n" );
    uint16_t inst = HD44790U_Inst::ret_home;
    mpu_interface( inst, check_BF );
    delay( 10 ); // Wait about 10 milliseconds.

    // Return the cursor position to {0,0}
    csr_pos = { 0, 0, true };
    disp_strt_pos = 0;
    return true;
    } // end ret_home( )

void LCD_Display::entry_m_set( const Csr_Dir csr_mv, const Disp_Shft_on_w shift_disp, const bool check_BF )
    {
    csr_dir = csr_mv;       
    shfts_w = shift_disp;
    uint16_t inst = HD44790U_Inst::entry_m_set;
    inst |= static_cast< uint16_t >( csr_mv ) << 1;
    inst |= static_cast< uint16_t >( shift_disp );
    mpu_interface( inst, check_BF );
    } // end ret_home( )


void LCD_Display::display_state_control( const bool disp_on, const bool csr_on, const bool csr_blinks )
    {
#ifdef DEBUG
    Serial.print( "In display_state_control\n" );
#endif
    uint16_t inst = HD44790U_Inst::disp_io_ctrl;
    inst |= ( disp_on ? 1 : 0 ) << 2;
    inst |= ( csr_on ? 1 : 0 ) << 1;
    inst |= ( csr_blinks ? 1 : 0 );
    mpu_interface( inst );
    } // end display_state_control( )

uint16_t shift_helper( const D_C d_or_c,  const Direction dir );
void LCD_Display::move_cursor( const Direction dir, const bool check_BF )
    {
    uint16_t inst = shift_helper( D_C::Cursor, dir ); 
    mpu_interface( inst, check_BF );

    Csr_Pos_t old_pos = csr_pos;
    update_csr_pos( dir );

    // If the cursor address moves right on 0x27, moves to 0x40
    // else if cursor address moves left on 0x00, moves to 0x67.
    // Therefore, since this is considered undesireable behavior,
    // we need to correct the cursor position.
    if ( disp_info & row_bit_c ) // Two lines mode
        {
        if ( old_pos.row == 0 ) 
            {
            // Need to do corrective measures in software
            if ( old_pos.pos == 39 && dir == Direction::Right )
                {
                Assert( jmp_abs_pos( { 0, 0 } ), "move_cursor jump to {0,0} failed\n" );
                } // end if
            else if ( old_pos.pos == 0 && dir == Direction::Left )
                {
                // Need to skip BF flag since we are no longer within 0 and 15 for pos.
                Assert( jmp_abs_pos( { 0, 39 }, false ), "move_cursor jump to {0,39} failed\n" );
                } // end else if
            } // end if
        } // end if
    } // end move_cursor( )

void LCD_Display::shift_display( const Direction dir )
    {
    assert( dir == Direction::Left || dir == Direction::Right );
    uint16_t inst = shift_helper( D_C::Display , dir ); 
    mpu_interface( inst );
    update_disp_strt_pos( dir );
    csr_pos.pos = csr_pos.pos + ( dir == Direction::Left ? -1 : 1 );
    if ( csr_pos.pos < lower_pos_c )
        {
        jmp_abs_pos( { csr_pos.row, 0 } ); // Readjust to zero
        } // end if
    else if ( csr_pos.pos == upper_rel_pos_c )
        {
        jmp_abs_pos( { csr_pos.row, upper_rel_pos_c - 1 } ); // Readjust to rightmost position.
        } // end else if
    } // end shift_display( )

uint16_t shift_helper( const D_C d_or_c, const Direction dir )
    {
    uint16_t inst = HD44790U_Inst::csr_disp_shft;
    inst |= static_cast< uint16_t > ( d_or_c ) << 3; // S/C (Display (S)hift / Move (C)ursor)
    inst |= static_cast< uint16_t >( dir ) << 2;     // R/L ((R)ight/(L)eft)
    return inst;
    } // end shift_helper( )

void LCD_Display::function_set( const MPU_bit_interf bit_interface, const enum func_set n_f, const bool check_BF )
    {
#ifdef DEBUG
    Serial.print( "In function_set\n" );
#endif
    uint16_t inst = HD44790U_Inst::func_set;
    inst |= static_cast< uint16_t > ( bit_interface ) << 4; // DL (Data Length)
    inst |= static_cast< uint16_t > ( n_f ) << 2;           // N+F (N)umber of DL + char (F)ont
    mpu_interface( inst, check_BF );

    set_bit_interface( bit_interface ); // Needs to be done after mpu_interface (Since it still requires prev interface)
    disp_info = n_f;
    } // end function_set( 0)

void LCD_Display::set_CGRAM_addr( const uint8_t addr_counter )
    {
    uint16_t inst = HD44790U_Inst::set_cgram_addr;
    inst |= addr_counter; // S/C
    mpu_interface( inst );
    } // end set_CGRAM_addr( )

void LCD_Display::set_DDRAM_addr( const uint8_t addr_counter, const bool check_BF )
    {
    Assert( !( disp_info & row_bit_c ) || 
        ( addr_counter < 40 || ( addr_counter >= 64 && addr_counter < 104 ) ),
        "2-line mode: addr_counter isn't within bounds" );
    Assert( ( disp_info & row_bit_c ) || ( addr_counter < 80 ),
        "1-line mode: addr_counter isn't withint bounds" );
    
    uint16_t inst = HD44790U_Inst::set_ddram_addr;
    inst |= addr_counter; // S/C
    mpu_interface( inst, check_BF );
    } // end set_DDRAM_addr( )

bool LCD_Display::is_busy( ) const
    {
    return read( ) & DB7;
    } // end is_busy( )

uint8_t LCD_Display::get_addr_counter( ) const
    {
    return read( ) & ( DB7 - 1 );
    } // end get_addr_counter( )

void LCD_Display::write_CG_DDRAM( const uint8_t data_byte, const bool check_BF )
    {
    uint16_t inst = HD44790U_Inst::wr_data_CG_DDRAM;
    inst |= data_byte;
    mpu_interface( inst, check_BF );
    if ( shfts_w == Disp_Shft_on_w::Yes )
        {
        // Moves right on decrement, and left on increment.
        update_disp_strt_pos( csr_dir == Csr_Dir::Increment ? 
                            Direction::Left : Direction::Right );
        } // end if
    } // end write_CG_DDRAM( )


uint8_t LCD_Display::read_CG_DDRAM( )
    {
    uint16_t inst = HD44790U_Inst::rd_data_CG_DDRAM;
    mpu_interface( inst );
    return static_cast< uint8_t >( inst );
    } // end read_CG_DDRAM( )

void LCD_Display::mpu_interface( uint16_t& bits, const bool check_BF )
    {
#ifdef DEBUG
    Serial.print( "Call mpu_interface: ");
    Serial.print( bits, BIN );
    Serial.print( "\n" );
#endif

    // assert( is_good( ) );
    const bool is_read = bits & rw_mask_c; // True if read, false if write

    if ( check_BF )
        {
        // uint8_t bf_seen = 0;
        while( /* bf_seen < 10 && */ is_busy( ) ) // Wait until BF==0 or seen bf too many times
            {
            Serial.print( "is_busy\n" );
            //++bf_seen;
            } // end while
        } // end if
    //! This is a workaround since we cannot guarantee that instructions won't be
    //! eaten since checking the BF flag outside a certain range cannot be done properly
    //! By simply waiting long enough this should ensure that most instructions are done.
    else 
        {
        delay( 1 );
        }  // end else

    if ( is_read )
        bits |= read( bits & rs_mask_c );
    else
        write( static_cast< uint8_t >( bits ), bits & rs_mask_c ); // Chop the lower 8 bits.
    
    MR_MPU = bits;
    } // end send_instr( )

static void write_four_db( const int8_t data_bus[ 4 ], const uint8_t data );
void LCD_Display::write( const uint8_t db0_7, const bool data )
    {
    set_db_io( OUTPUT ); // Set pin mode appropiately
    digitalWrite( rs, data ? HIGH : LOW ); // Send whether is read or write
    digitalWrite( rw, LOW ); // To signify write
    delayMicroseconds( 1 ); // tsu1 == 100 ns (RS, R/W setup time)
    digitalWrite( en, HIGH ); // Pull up enable

#ifdef DEBUG
    Serial.print( "Sending for DB7 - DB4: 0b" );
    Serial.print( db0_7 >> 4, BIN );
    Serial.print( "\n" );
#endif

    write_four_db( data_bus + 4, db0_7 >> 4 ); // Send db4 - db7
    if ( interface_m == MPU_bit_interf::Four )
        {
        // Pull down enable and up again
        // Setup (tDSW == 80ns )
        // delayMicroseconds( 1 );
        digitalWrite( en, LOW ); // Pull down enable
        // Hold time (hH == 10ns ) -> Way too low to be of concern
        // delayMicroseconds( 1 );     // th_min (10ns)
        digitalWrite( en, HIGH ); // Pull up enable
        write_four_db( data_bus + 4, db0_7 ); // Send db0 - db3
        } // end if
    else
        {
        write_four_db( data_bus, db0_7 ); // Send db0 - db3
        } // end else
#ifdef DEBUG
    Serial.print( "Sending for DB0 - DB3: 0b" );
    Serial.print( db0_7 & ( ( 1 << 4 ) - 1 ), BIN );
    Serial.print( "\n" );
#endif

    // delayMicroseconds( 1 );
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

#define TDDR 1

static uint8_t read_four_db( const int8_t data_bus[ 4 ] );
uint8_t LCD_Display::read( const bool data ) const
    {
    set_db_io( INPUT ); // Set pin mode to in to read
    digitalWrite( rs, data ? HIGH : LOW ); // Send whether is read or write
    digitalWrite( rw, HIGH ); // To signify read
    // delayMicroseconds( 1 ); // tsu1 == 100 ns (RS, R/W setup time)
    digitalWrite( en, HIGH ); // Pull up enable
    uint8_t db;
    int8_t const *db_ptr = data_bus + 4;

    // Max Delay is 360ns (tDDR)
    delayMicroseconds( TDDR );
    db = read_four_db( db_ptr ); // Read in DB4 - DB7

#ifdef DEBUG
    Serial.print( "For DB4 - DB7 got: 0b" );
    Serial.print( db, BIN );
    Serial.print( "\n" );
#endif
    if ( interface_m == MPU_bit_interf::Four )
        {
        // Pull down enable and up again
        digitalWrite( en, LOW ); // Pull down enable
        delayMicroseconds( 1 );     // th_min (10ns)
        digitalWrite( en, HIGH ); // Pull up enable
        delayMicroseconds( TDDR ); // Wait until tDDR_max passes (360ns)
        } // end if
    else
        {
        db_ptr = data_bus; // To the read the bottom 4 wires in 8-bit mode.
        } // end else
    db <<= 4;
    db |= read_four_db( data_bus ); // Read in DB0 - DB3

#ifdef DEBUG
    Serial.print( "For DB3 - DB0 got: 0b" );
    Serial.print( db >> 4, BIN );
    Serial.print( "\n" );
#endif

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
        db_ptr != data_bus + ( sizeof( data_bus ) / sizeof( int8_t ) ); ++db_ptr )
        {
        pinMode( *db_ptr, pin_mode );
        } // end for
    } // end set_db_io( )