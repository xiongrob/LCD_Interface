#include "LCD_HD44780.hh"

LCD_Display lcd_disp;
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
    delay( 500 );
    } // end for
  lcd_disp.write_CG_DDRAM( 'A', false );
  lcd_disp.write_CG_DDRAM( 'B', false );
  Serial.print( lcd_disp.is_busy( ) ? "Is busy!\n" : "Not busy\n" );
  lcd_disp.ret_home( false );
  Serial.print( lcd_disp.is_busy( ) ? "Is busy!\n" : "Not busy\n" );
  for ( int n = 0; n < 8; ++n )
    {
    lcd_disp.shift_display( Direction::Left );
    delay( 500 );
    } // end for
  lcd_disp.ret_home( );
  lcd_disp.set_DDRAM_addr( 0xC0 );
  delay( 5000 );
  lcd_disp.set_DDRAM_addr( 0x40 );
  delay( 5000 );
  lcd_disp.shift_display( Direction::Right );
  lcd_disp.set_DDRAM_addr( 0x67 );
} // end setup( )

void loop() 
    {
    // put your main code here, to run repeatedly:
    }
