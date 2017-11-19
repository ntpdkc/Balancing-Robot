//=========================================================================================================
//Serial1 setting
inline void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // Always store new byte on ring buffer.
  buffer->buffer[buffer->head] = c;
  buffer->head = i;
  //  Overflow? => Discard old bytes
  if (i == buffer->tail) {
    buffer->tail = (unsigned int)(buffer->tail + 1) % SERIAL_BUFFER_SIZE;  // Move tail to discard old bytes
    rx_bufferS1_overflow = 1;                                              // Buffer overflow flag
  }
}

ISR(USART1_RX_vect)
{
  if (bit_is_clear(UCSR1A, UPE1)) {
    unsigned char c = UDR1;
    store_char(c, &rx_bufferS1);
    if (Serial1_available() > 0) {
      int inchar = Serial1_read();
      if (inchar != '\n') {
        ser1 += (char)inchar;
      }
      else {
        if (ser1.substring(0, 3) == "st@") {
          setting = ser1.substring(3, 11);
        }
        else if(ser1.substring(0,3) == "sv@"){
          servo = ser1.substring(3,5);
        }
        ser1 = "";
      }
    }
  } else {
    unsigned char c = UDR1;
  };
}

int Serial1_available(void)
{
  return (unsigned int)(SERIAL_BUFFER_SIZE + rx_bufferS1.head - rx_bufferS1.tail) % SERIAL_BUFFER_SIZE;
}

unsigned char Serial1_read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_bufferS1.head == rx_bufferS1.tail) {
    return -1;
  } else {
    unsigned char c = rx_bufferS1.buffer[rx_bufferS1.tail];
    rx_bufferS1.tail = (unsigned int)(rx_bufferS1.tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
  }
}

void Serial1_write(uint8_t c)
{
  while (!((UCSR1A) & (1 << UDRE1)))
    ;
  UDR1 = c;
}

void Serial1_flush()
{
  rx_bufferS1.head = rx_bufferS1.tail;
}

void Serial1_print(const char str[])
{
  while (*str)
    Serial1_write(*str++);
}

void Serial1_println(const char str[])
{
  while (*str)
    Serial1_write(*str++);
  Serial1_write('\r');
  Serial1_write('\n');
}
void Serial1_begin(unsigned long baud)
{
  uint16_t baud_setting;

  UCSR1A = (1 << U2X1);
  baud_setting = (F_CPU / 4 / baud - 1) / 2;

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  UBRR1H = baud_setting >> 8;
  UBRR1L = baud_setting;

  //transmitting = false;
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
}
//=========================================================================================================
