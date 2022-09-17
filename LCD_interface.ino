#include "LCD_HD44780.hh"

LCD_Display lcd_disp;

/*
void setup() {
  Serial.begin( 9600 );
  // put your setup code here, to run once:
  //int8_t db[ 8 ] = { -1, -1, -1, -1, 5, 6, 7, 8 };
  int8_t db[ 8 ] = { 9, 10, 11, 12, 5, 6, 7, 8 };
  Serial.print( "After db\n" );
  lcd_disp.attach_LCD_display( db, 2, 3, 4, MPU_bit_interf::Eight, two_line_5x8 );
  lcd_disp.init_LCD( Csr_Dir::Increment, Disp_Shft_on_w::No, true, true );
  lcd_disp.clear_display( );
  Serial.print( "Got here\n" );
  lcd_disp.display_state_control( true, true, true );
  lcd_disp.entry_m_set( Csr_Dir::Increment, Disp_Shft_on_w::No );
  lcd_disp.ret_home( );
  lcd_disp.write_CG_DDRAM( 'H' );
  lcd_disp.write_CG_DDRAM( 'I' );
  lcd_disp.write_CG_DDRAM( 'T' );
  lcd_disp.write_CG_DDRAM( 'A' );
  lcd_disp.write_CG_DDRAM( 'C' );
  lcd_disp.write_CG_DDRAM( 'H' );
  lcd_disp.write_CG_DDRAM( 'I' );
  lcd_disp.set_DDRAM_addr( 15 );
  // lcd_disp.set_DDRAM_addr( 16 );
  Serial.print( "Waiting...\n" );
  delay( 5000 );
  //lcd_disp.write_CG_DDRAM( 'Z' );
  lcd_disp.ret_home( );
  Serial.print( lcd_disp.get_addr_counter( ) );
  Serial.print( "\n" );
  lcd_disp.set_DDRAM_addr( 0xC0 );
  lcd_disp.set_DDRAM_addr( 0x00 );
  Serial.print( lcd_disp.read_CG_DDRAM( ) );
  Serial.print( "\n" );

  lcd_disp.shift_display( Direction::Right );
  lcd_disp.move_cursor( Direction::Left );
  lcd_disp.set_DDRAM_addr( 0x67 );

  Serial.print( lcd_disp.get_addr_counter( ) );
  Serial.print( "\n" );
  lcd_disp.ret_home( );
  lcd_disp.clear_display( );

  for ( int n = 0; n < 32; ++n )
    {
    if ( n == 15 )
        {
        delay( 1000 );
        lcd_disp.entry_m_set( Csr_Dir::Decrement, Disp_Shft_on_w::No );
        }
    else if( n == 16 )
        {
        lcd_disp.set_DDRAM_addr( 0xC0 );
        lcd_disp.entry_m_set( Csr_Dir::Increment, Disp_Shft_on_w::No );
        } // end if
    else if ( n == 31 )
        {
        delay( 1000 );
        lcd_disp.entry_m_set( Csr_Dir::Decrement, Disp_Shft_on_w::No );
        } // end if
    lcd_disp.write_CG_DDRAM( 'A' + ( n % 26 ) );
    Serial.print( lcd_disp.get_addr_counter( ), HEX );
    Serial.print( "\n" );
    } // end for

  lcd_disp.entry_m_set( Csr_Dir::Increment, Disp_Shft_on_w::No );
  for ( int n = 0; n < 8; ++n )
    {
    lcd_disp.shift_display( Direction::Left );
    } // end for

  lcd_disp.set_DDRAM_addr( 0xC0 );
  for ( int n = 0; n < 16; ++n )
    {
    lcd_disp.move_cursor( Direction::Right );
    } // end for
  lcd_disp.write_CG_DDRAM( 'A', false );
  lcd_disp.write_CG_DDRAM( 'B', false );
  Serial.print( lcd_disp.is_busy( ) ? "Is busy!\n" : "Not busy\n" );
  lcd_disp.ret_home( false );
  Serial.print( lcd_disp.is_busy( ) ? "Is busy!\n" : "Not busy\n" );
  for ( int n = 0; n < 8; ++n )
    {
    lcd_disp.shift_display( Direction::Left );
    delay(500);
    } // end for
  lcd_disp.ret_home( );
  lcd_disp.move_cursor( Direction::Right, false );
  delay( 2000 );
  lcd_disp.shift_display( Direction::Right );

  Serial.print( "Passing 40th digit\n");
  delay( 500 );
  lcd_disp.move_cursor( Direction::Left, false ); //Passes 40th digit
  delay( 500 );
  lcd_disp.shift_display( Direction::Right );
  delay( 500 );
  lcd_disp.move_cursor( Direction::Left, false );
  lcd_disp.ret_home( );
  for ( int n = 0; n < 39; ++n )
    {
    if ( n == 15 )
      lcd_disp.entry_m_set( Csr_Dir::Increment, Disp_Shft_on_w::Yes );
    lcd_disp.write_CG_DDRAM( 'A' + ( n % 26 ), false );
    } // end for
    lcd_disp.shift_display( Direction::Left );
    lcd_disp.shift_display( Direction::Left );
    lcd_disp.shift_display( Direction::Left );
    lcd_disp.shift_display( Direction::Left );
    delay( 5000 );
    lcd_disp.move_cursor( Direction::Left );
    delay( 5000 );
    lcd_disp.move_cursor( Direction::Right );
    delay( 5000 );
    lcd_disp.move_cursor( Direction::Right );
    delay( 5000 );
    lcd_disp.set_DDRAM_addr( 0 );
    delay( 5000 );
    lcd_disp.move_cursor( Direction::Left );
    delay( 5000 );
    lcd_disp.move_cursor( Direction::Left );
} // end setup( )
*/
/// Register Select (RS) Pin for LCD
const int RS  = 2;
/// Read Write (RW) Pin for LCD
const int RW  = 4;
/// Enable Pin for LCD
const int EN_LCD = 6;

/// DB7-DB0 Pins 
const int8_t DBX[ 8 ] = { -1, -1, -1, -1, 14, 15, 16, 17 }; // Use A0-A3

void setup() {
  Serial.begin( 9600 );
  // put your setup code here, to run once:
  //int8_t db[ 8 ] = { -1, -1, -1, -1, 5, 6, 7, 8 };
  int8_t db[ 8 ] = { 9, 10, 11, 12, 5, 6, 7, 8 };
  //int8_t db[ 8 ] = { -1, -1, -1, -1, 5, 6, 7, 8 };
  Serial.print( "After db\n" );
  lcd_disp.attach_LCD_display( DBX, RS, RW, EN_LCD, MPU_bit_interf::Four, two_line_5x8 );
  //lcd_disp.attach_LCD_display( db, 2, 3, 4, MPU_bit_interf::Four, two_line_5x8 );
  lcd_disp.init_LCD( Csr_Dir::Increment, Disp_Shft_on_w::No, true, true );
  Serial.print( "Got here\n" );
  
  lcd_disp.write_CG_DDRAM( 'H' );
  lcd_disp.write_CG_DDRAM( 'I' );
  lcd_disp.write_CG_DDRAM( 'T' );
  lcd_disp.write_CG_DDRAM( 'A' );
  lcd_disp.write_CG_DDRAM( 'C' );
  lcd_disp.write_CG_DDRAM( 'H' );
  lcd_disp.write_CG_DDRAM( 'I' );
  delay( 1000 );
  lcd_disp.clear_display( );
  lcd_disp.put_char( 'H' );
  lcd_disp.put_char( 'I' );
  lcd_disp.put_char( '!' );
  lcd_disp.get_csr_pos( ).print( );
  lcd_disp.shift_display( Direction::Right );
  lcd_disp.goto_next_ln( );
  lcd_disp.get_csr_pos( ).print( );

  lcd_disp.set_csr_bounds( Cursor_Bounds::Ln_Br );
  delay( 2000 );
  lcd_disp.ret_home( );
  lcd_disp.type_chars( "Arduino Rules!!!" );
  Serial.print( "done\n" );
  // lcd_disp.clear_display( false );
}
void loop() 
    {
    // put your main code here, to run repeatedly:
    //
    lcd_disp.type_chars( "Roomba", { 1, 0 } );
    delay( 2000 );
    lcd_disp.clear_row( 1 );
    lcd_disp.type_chars( "BLE 4ever", { 1, 0 } );
    delay( 2000 );
    lcd_disp.clear_row( 1 );
    //
    /*
    lcd_disp.clear_display( );
    lcd_disp.type_chars( "Arduino Rules!!!Roomba" );
    delay( 2000 );
    lcd_disp.clear_display( );
    lcd_disp.type_chars( "Arduino Rules!!!BLE 4ever" );
    delay( 2000 );
    */
    }
