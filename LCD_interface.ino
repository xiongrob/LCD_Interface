#include "LCD_HD44780.hh"

LCD_Display lcd_disp;
void setup() {
  Serial.begin( 9600 );
  // put your setup code here, to run once:
  //int8_t db[ 8 ] = { -1, -1, -1, -1, 5, 6, 7, 8 };
  int8_t db[ 8 ] = { 9, 10, 11, 12, 5, 6, 7, 8 };
  Serial.print( "After db\n" );
  lcd_disp.attach_LCD_display( db, 2, 3, 4, MPU_bit_interf::Four, two_line_5x8 );
  lcd_disp.init_LCD( Move_csr::Increment, Disp_Shft_on_w::No, true, true );
  lcd_disp.clear_display( );
  Serial.print( "Got here\n" );
  lcd_disp.display_state_control( true, true, true );
  lcd_disp.entry_m_set( Move_csr::Increment, Disp_Shft_on_w::No );
  lcd_disp.ret_home( );
  lcd_disp.write_CG_DDRAM( 'H' );
  lcd_disp.write_CG_DDRAM( 'I' );
  lcd_disp.write_CG_DDRAM( 'T' );
  lcd_disp.write_CG_DDRAM( 'A' );
  lcd_disp.write_CG_DDRAM( 'C' );
  lcd_disp.write_CG_DDRAM( 'H' );
  lcd_disp.write_CG_DDRAM( 'I' );
  lcd_disp.set_DDRAM_addr( 15 );
  //lcd_disp.set_DDRAM_addr( 16 );
  Serial.print( "Waiting...\n" );
  delay( 3000 );
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

  for ( int n = 0; n < 16; ++n )
    {
    lcd_disp.write_CG_DDRAM( 'A' + n );
    delay( 1000 );
    } // end for

  // lcd_disp.entry_m_set( Move_csr::Increment, Disp_Shft_on_w::No );
}

void loop() 
    {
    // put your main code here, to run repeatedly:
    }
