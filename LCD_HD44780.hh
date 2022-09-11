

#ifndef LCD_HD44780_H
#define LCD_HD44780_H
#include <stdint.h> // for uint8_t
#include "assert.h"

// If Four is selected, data_bus[4:7] are ignored.
enum class MPU_bit_interf { Four, Eight };

enum class Csr_Dir { Decrement, Increment, None };
enum class Disp_Shft_on_w { No, Yes };
enum class D_C            { Cursor, Display };
enum class Direction      { Left, Right, Up, Down };
enum class IO_state       { Input, High };
enum class Cursor_Bounds  { Stops, Continues, Ln_Br };


// Low level instructions
// Code: <RS | R/!W | DB7 | DB6 | DB5 | DB4 | DB3 | DB2 | DB1 | DB0>
enum HD44790U_Inst
    {
    // Instruction(1)
    clear_display    = 0b0000000001, // Clear display
    ret_home         = 0b0000000010, // Return home
    entry_m_set      = 0b0000000100, // Entry mode set
    disp_io_ctrl     = 0b0000001000, // display on/off control
    csr_disp_shft    = 0b0000010000, // Cursor Display Shift
    func_set         = 0b0000100000, // Function Set
    set_cgram_addr   = 0b0001000000, // Set CGRAM address (CGRAM data is sent and received after this setting.)
    // Instruction(2)
    set_ddram_addr   = 0b0010000000, // Set CGRAM address (DDRAM data is sent and received afte rthis setting.)
    read_BF_addr     = 0b0100000000, // Set CGRAM address
    // Instruction(3)
    wr_data_CG_DDRAM = 0b1000000000, // Write data to CG or DDRAM ()
    rd_data_CG_DDRAM = 0b1100000000, // Read data from CG or DDRAM
    };

enum Data_Bit
    {
    DB0 = 0b00000001, 
    DB1 = 0b00000010,
    DB2 = 0b00000100, 
    DB3 = 0b00001000, 
    DB4 = 0b00010000, 
    DB5 = 0b00100000, 
    DB6 = 0b01000000, 
    DB7 = 0b10000000, 
    };


// S/C R/L
enum shift_func
    {
    shift_pos_left  = 0b0000,
    shift_pos_right = 0b0100,
    shift_disp_left = 0b1000,
    shift_disp_right= 0b1000,
    };

/// N(Number of display lines) F(char Font)
enum func_set
    {
    one_line_5x8    = 0b00, // 1 display line 5x8  character font
    one_line_5x10   = 0b01, // 1 display line 5x10 character font
    two_line_5x8    = 0b10, // 2 display lines 5x8  character font
    };

const int row_bit_c = 1 << 1;
const int dl_num_c = 8;
const int lower_pos_c  = 0;
const int upper_pos_c  = 16;

struct Csr_Pos_t
    {
    int8_t row;
    int8_t pos;

    bool valid( const bool line_num = 2 ) const
        {
        return row >= 0 && row < line_num
            && pos >= 0 && pos < upper_pos_c;
        } // end valid( )
    };

enum class Cursor_Addr_Bounds
    {
    first_lower    = 0x00, // Number of lines 1
    first_upper_N1 = 0x4F, // Number of lines 2
    first_upper_N2 = 0x27, // Number of lines 2
    sec_lower      = 0x40,
    sec_upper      = 0x67,
    };

const int display_upper_c = 40; // After 40 lines, goes to next line.
const uint8_t disp_offset_c[ 2 ] = { 0x0, 0x40 };

//---------------------------------------------------------------------------------------------------
//
//                                     LCD_Display Declarations
//
//---------------------------------------------------------------------------------------------------
class LCD_Display
    {
    int8_t  data_bus[ 8 ];
    int8_t  rs : 6;
    int8_t  rw : 6;
    int8_t  en : 6; // enable
    int8_t  v0 : 6; // Used for changing contrast (-1 if using potentiometer). -> Make sure is analog pin
    MPU_bit_interf interface_m = MPU_bit_interf::Eight; // If Four is selected, data_bus[4:7] are ignored.

    uint16_t MR_MPU = 0; // Most recent sent inst (copied when send_inst was called) -> Used for checking state

    // LCD Display State
    enum func_set disp_info; // Display Information (like display lines and character font)
    Csr_Dir csr_dir;        // What direction does cursor move on write.
    Disp_Shft_on_w shfts_w;  // Does the Display move while writing characters?
    Csr_Pos_t csr_pos = { 0, 0 }; // Starts at row zero, column zero.
    uint8_t disp_strt_pos = 0; // Ranges from 0 to 39 (Used in combination of cursor position to calculate absolute position of cursor).
    Cursor_Bounds csr_bounds = Cursor_Bounds::Continues;
    uint8_t d_c_b; // Display, cursor, and display respectively
public:
    LCD_Display( ) : LCD_Display( nullptr, -1, -1, -1, MPU_bit_interf::Eight ) 
        { 
        } // end LCD_Display( )

    LCD_Display( const int8_t data_bus[ 8 ], const int rs, const int rw, const int e, const MPU_bit_interf );

    void print_pins( ) const;

    // Used for setting everything
    // the upper 8 are ignored if 4-bit interface is used
    bool attach_LCD_display( const int8_t _data_bus[ 8 ], const uint8_t rs, const uint8_t rw, const uint8_t e, const MPU_bit_interf, const enum func_set n_f );

    void init_LCD( const Csr_Dir csr_mv, const Disp_Shft_on_w shift_disp, const bool csr_on = false, const bool csr_blinks = false );
    void init_LCD( const MPU_bit_interf bit_inter, const enum func_set n_f, const Csr_Dir csr_mv, const Disp_Shft_on_w shift_disp, 
                            const bool csr_on, const bool csr_blinks );

    // Setters
    // Put -1 if don't want to change.
    void set_data_bus( const int8_t data_bus_pins [ 8 ] );

    void set_register_sel( const uint8_t rs_pin );

    void set_read_write( const uint8_t rw_pin );

    void set_enable( const uint8_t e_pin );

    void set_bit_interface( const MPU_bit_interf bit_inter );

    // Getters
    uint8_t get_db( const uint8_t db_num ) const { return data_bus[ db_num ]; }
    int8_t const * get_db_arr( ) const { return data_bus; }
    uint8_t get_rw( ) const { return rw; }
    uint8_t get_enable( const uint8_t db_num ) const { return en; }
    uint8_t get_v0( ) const { return v0; }
    MPU_bit_interf get_bit_interface( ) const { return interface_m; }

    // Checks that pins are fine
    bool is_good( ) const;

    //---------------------------------------------------------------------------------------------------
    //
    //                                     Top-Level Interface Declarations
    //
    //---------------------------------------------------------------------------------------------------    

    /*
     * Types Characters to Display 
     * EFFECTS: Sends characters in c_string to display (starting at cursor).
     */
    bool type_chars( const char *c_string, const Csr_Pos_t crsr_pos = { -1, -1 } );

    /*
     * Puts Charcter to Display
     * EFFECTS: Sends character to display at cursor it is (autoincrement will occur accordenly)
     * Will either go to the next line at end or continue to shift display (as you go if so).
     */
    void put_char( const char character );

    /*
     * Jumps to row and position on display.
     * EFFECTS: Moves cursor to certain location on display.
     *          Returns true or false depending on whether or not 
     *          it succeeded or not.
     */
    bool jmp_pos( const Csr_Pos_t csr_pos );


    /*
     * Clear Row
     * EFFECTS: Clears entire row based on row number (all 40 digits).
     */
    void clear_row( const int8_t row_num );  // Clears row.
    void shift_cursor ( const Direction dir ); // Up Down left, right.
    void display_ctrl( const bool on );
    void csr_ctrl( const bool crsr_on, const bool blink );

    /*
     * Toggle Display (Display On/Off Control)
     * EFFECTS: display is on when on is true, and off if off.
     */
    void toggle_display( );

    void toggle_cursor( );
    void toggle_cursor_blink( );

    // Backdoor Functions ->
    // These functions will serve as helper functions that provides the interface provided by the manufactuers to help implement 
    // the interface and will be allowed for the programmers to take advantage of (if they so please).

    /* Clear Display
     * EFFECTS: Writes space code 0x20 into all DDRAM addresses, sets DDARM addr 0 into AC (address counter),
     *          and returns the display to its original status (if shifted).
     */
    bool clear_display( const bool check_BF = true );

    /* Return Home
     * EFFECTS: Sets DDRAM address 0 into address counter, and returns the display to its original status if shifted.
     *          The DDRAM contents do not change.
     */
    bool ret_home( const bool check_BF = true );

    /*
     * Entry Mode Set
     * EFFECTS: By setting Increment cursor on write, will move the cursor right whenever a write occurs for address 
     *          written to either DDRAM or CGRAM.
     *          Also shifts the entire display right when set to decrement or right when incrementing. 
     */
    void entry_m_set( const Csr_Dir csr_mv, const Disp_Shft_on_w shift_disp );


    /*
     * Display State Control (Display On/Off Control)
     * EFFECTS: Turns display on or off (depending on disp_on), cursor being displayed with csr_on,
     *          and whether or not cursor blinks (csr_blinks).
     */
    void display_state_control( const bool disp_on, const bool csr_on, const bool csr_blinks );

    /* 
     * Move Cursor
     * EFFECTS: Moves cursor left or right (which consequencly changes address counter)
     * Cursor moves to the second line when it passes the 40th digit of the first line.
     */
    void move_cursor  ( const Direction dir );
    /* 
     * Shift Display
     * EFFECTS: Shifts display left or right (doesn't change AC like move_cursor)
     * aursor moves to the second line when it passes the 40th digit of the first line.
     * Both first and second line displays shift at the same time.
     */
    void shift_display( const Direction dir );

    /* 
     * Function Set
     * EFFECTS: Sets interface data length (to either 8-bit (DB0-DB7) or 4-bit lengths (DB7 to DB4) which requires data to be sent twice).
     *          as well as the set the number of display lines and character font.
     * ***Note: Number of display lines (N) and character font (F) can only be set once after interface data length is changed.
     *          Also perform before executing any instructions (EXECPT is_busy or reading address instructions).
     */
    void function_set( const MPU_bit_interf bit_interface, const enum func_set n_f, const bool check_BF = false );

    /* 
     * Set CGRAM Address
     * EFFECTS: Sets CGRAM address binary into AC (address counter).
     * Afterwards any data that's writen to or read from is from CGRAM.
     */
    void set_CGRAM_addr( const uint8_t addr_counter );

    /* 
     * Set DDRAM Address
     * EFFECTS: Sets DDRAM address binary into AC (address counter).
     * Afterwards any data that's writen to or read from is from DDRAM.
     * 
     * Note: Useful for moving cursor around DDRAM.
     *       Though when N is 0 (1-line display) can only write from 0x00 to 0x4F.
     *       When N is 1 (2-line display), can write from 00H to 0x27 for the first line
     *       and 0x40 to 0x67 for the second.
     */
    void set_DDRAM_addr( const uint8_t addr_counter );

    // Checks if BF is high (currently in an internal operation). -> This is in case programmer wants to do somethign else.
    /*
     * Is Busy (Read Busy Flag)
     * Check if system is interally operating on previously received instruction.
     * 
     * Important since next instruction won't be accepted until BF (busy flag) is reset to 0.
     */
    bool is_busy( ) const; 

    /*
     * Get Address Counter
     * Gets value of value counter determed by previous instruction (in case set occured).
     */
    uint8_t get_addr_counter( ) const;

    /*
     * Write CG or DDRAM.
     * Write 8-bit binary data to either CG or DDRAM (Also address will automatically increment 
     * or decrement depending on entry mode).
     * 
     * This is for writing characters to LCD.
     */
    void write_CG_DDRAM( const uint8_t data_byte, const bool check_BF = true );

    /*
     * Read CG or DDRAM.
     * Reads 8-bit binary data to from CG or DDRAM (Set must be executed or else first read data will be invalid).
     * After read, address will automatically incremen or decrement depending on entry mode).
     * 
     * Rule of thumb, to correct read data, either execute address set instruction or cursor shift instruction (for DDRAM) before reading.
     */
    uint8_t read_CG_DDRAM( );

    /*
     * MPU Interface 
     * Reads in a 10-bit encoding of instruction / data read write: 
     * <RS | R/!W | DB7 | DB6 | DB5 | DB4 | DB3 | DB2 | DB1 | DB0>
     * EFFECTS: Considers the 10 LSBs and uses them to set the
     * RS, rw, and data bus according.
     * 
     * If check_BF is true, waits for BF to be zero before writing / reading bits.
     * 
     * Occasionally BF checks needs to be skipped due to initialization and scenarios where
     * BF is always high (due to the cursor being at a particular address).
     * 
     * Note: Not designed for users to call it directly as it puts the LCD_Display class in some
     *       undefined state.
     */
    void mpu_interface( uint16_t& bits, const bool check_BF = true ); // Generic which sends everything (may return bits for reading)

    /*
     * Write to MPU interface
     * EFFECTS: Assumes a write to either the IR (instruction register) or DR (data register) -> according to whether data is false 
     *          or true respectively. Transfers DB7-DB0 through db0_7.
     */
    void write( const uint8_t db0_7, const bool data = false );
    /*
     * Read to MPU interface
     * EFFECTS: Assumes a read from either the Address Counter and BF (busy flag) or the CG or DDRAM -> according to whether data is false 
     *          or true respectively. Reads DB7-DB0 and returns a uint8 of it where DB7 is the MSB and DB0 the LSB.
     */
    uint8_t read( const bool data = false ) const; // Assumes that is reading.

    /*
     * Set Data Base I/O
     * EFFECTS: Sets the pins at db_pins to be either INPUT or OUTPUT depending on pin_mode.
     *          This is so they can be properly configured to read or write to the MPU interface.
     */
    void set_db_io( const int pin_mode ) const;
    }; // end LCD_Display

// Erases number of characters from display from cursor.
// Essentially sends entry mode instruction 

enum class Cursor_state
    {
    Normal,
    Edge_Dec,
    Edge_Inc,
    };

#endif
