//******************************************************************************
// ������� ������ I2C ���������������� STM8S003F
//
// �����: ������ ������
// ����:  17 ���� 2014
// URL:   http://hamlab.net/
//******************************************************************************

#include "i2c_drv.h"
#include <intrinsics.h>
#include <iostm8.h>

//������� �������� ������� I2C
static unsigned long int i2c_timeout;

//������ ������� � �������������
#define set_tmo_us(time)  i2c_timeout = (unsigned long int)(F_MASTER_MHZ * time)

//������ ������� � �������������
#define set_tmo_ms(time)  i2c_timeout = (unsigned long int)(F_MASTER_MHZ * time * 1000)

#define tmo               i2c_timeout--

//�������� ����������� ������� event
//� ������� ������� timeout � ��
#define wait_event(event, timeout) set_tmo_ms(timeout);\
                                   while(event && --i2c_timeout);\
                                   if(!i2c_timeout) return I2C_TIMEOUT;
                                   
                                   
//******************************************************************************
// ������������� I2C ����������      
//      f_master_hz - ������� ������������ ��������� Fmaster                                   
//      f_i2c_hz - �������� �������� ������ �� I2C                                   
//******************************************************************************                                   
void i2c_master_init(unsigned long f_master_hz, unsigned long f_i2c_hz){
  unsigned long int ccr;
  
  PE_DDR_bit.DDR1 = 0;
  PE_DDR_bit.DDR2 = 0;
  PE_ODR_bit.ODR2 = 1;  //SDA
  PE_ODR_bit.ODR1 = 1;  //SCL
  
  PE_CR1_bit.C11 = 0;
  PE_CR1_bit.C12 = 0;
  
  PE_CR2_bit.C21 = 0;
  PE_CR2_bit.C22 = 0;
  
  //������� ������������ ��������� MHz
  I2C_FREQR_FREQ = 12;
  //��������� I2C
  I2C_CR1_PE = 0;
  //� ����������� ������ �������� I2C max = 100 ����/�
  //�������� ����������� ����� 
  I2C_CCRH_F_S = 0;
  //CCR = Fmaster/2*Fiic= 12MHz/2*100kHz
  ccr = f_master_hz/(2*f_i2c_hz);
  //Set Maximum Rise Time: 1000ns max in Standard Mode
  //= [1000ns/(1/InputClockFrequencyMHz.10e6)]+1
  //= InputClockFrequencyMHz+1
  I2C_TRISER_TRISE = 12+1;
  I2C_CCRL = ccr & 0xFF;
  I2C_CCRH_CCR = (ccr >> 8) & 0x0F;
  //�������� I2C
  I2C_CR1_PE = 1;
  //��������� ������������� � ����� �������
  I2C_CR2_ACK = 1;
}
//******************************************************************************
// ������ data slave-����������
//******************************************************************************                                   
t_i2c_status i2c_wr_data(unsigned char address, char * data){                                  
                                
  //���� ������������ ���� I2C
  wait_event(I2C_SR3_BUSY, 10);
    
  //��������� �����-�������
  I2C_CR2_START = 1;
  //���� ��������� ���� SB
  wait_event(!I2C_SR1_SB, 1);

  //���������� � ������� ������ ����� �������� ����������
  I2C_DR = address & 0xFE;
  //���� ������������� �������� ������
  wait_event(!I2C_SR1_ADDR, 1);
  //������� ���� ADDR ������� �������� SR3
  I2C_SR3;
    
    //���� ������������ �������� ������
    wait_event(!I2C_SR1_TXE, 1);
    //���������� ����� ��������
    I2C_DR = *data;

  //����� ������, ����� DR ����������� � ������ ������ � ��������� �������
  wait_event(!(I2C_SR1_TXE && I2C_SR1_BTF), 1);
  
  //�������� ����-�������
  I2C_CR2_STOP = 1;
  //���� ���������� ������� ����
  wait_event(I2C_CR2_STOP, 1);
  
  return I2C_SUCCESS;
}
//******************************************************************************
// ������ �������� slave-����������
//******************************************************************************                                   
t_i2c_status i2c_wr_reg(unsigned char address, unsigned char reg_addr,
                              char * data, unsigned char length){                                  
                                
  //���� ������������ ���� I2C
  wait_event(I2C_SR3_BUSY, 10);
    
  //��������� �����-�������
  I2C_CR2_START = 1;
  //���� ��������� ���� SB
  wait_event(!I2C_SR1_SB, 1);
  
  
  //���������� � ������� ������ ����� �������� ����������
  I2C_DR = address & 0xFE;
  //���� ������������� �������� ������
  wait_event(!I2C_SR1_ADDR, 1);
  //������� ���� ADDR ������� �������� SR3
  I2C_SR3;
  
  
  //���� ������������ �������� ������
  wait_event(!I2C_SR1_TXE, 1);
  //���������� ����� ��������
  I2C_DR = reg_addr;
  
  //�������� ������
  while(length--){
    //���� ������������ �������� ������
    wait_event(!I2C_SR1_TXE, 1);
    //���������� ����� ��������
    I2C_DR = *data++;
  }
  
  //����� ������, ����� DR ����������� � ������ ������ � ��������� �������
  wait_event(!(I2C_SR1_TXE && I2C_SR1_BTF), 1);
  
  //�������� ����-�������
  I2C_CR2_STOP = 1;
  //���� ���������� ������� ����
  wait_event(I2C_CR2_STOP, 1);
  
  return I2C_SUCCESS;
}

//******************************************************************************
// ������ �������� slave-����������
// Start -> Slave Addr -> Reg. addr -> Restart -> Slave Addr <- data ... -> Stop 
//******************************************************************************                                   
t_i2c_status i2c_rd_reg(unsigned char address, unsigned char reg_addr,
                              char * data, unsigned char length){
  
  //���� ������������ ���� I2C
  wait_event(I2C_SR3_BUSY, 10);
    
  //��������� ������������� � ����� �������
  I2C_CR2_ACK = 1;
  
  //��������� �����-�������
  I2C_CR2_START = 1;
  //���� ��������� ���� SB
  wait_event(!I2C_SR1_SB, 1);
  
  //���������� � ������� ������ ����� �������� ����������
  //I2C_DR = address & 0xFE;
  I2C_DR = (address << 1) | 1;
  //���� ������������� �������� ������
  wait_event(!I2C_SR1_ADDR, 1);
  //������� ���� ADDR ������� �������� SR3
  I2C_SR3;
  
  //���� ������������ �������� ������ RD
  wait_event(!I2C_SR1_TXE, 1);
  
  //�������� ����� �������� slave-����������, ������� ����� ���������
  I2C_DR = reg_addr;
  //����� ������, ����� DR ����������� � ������ ������ � ��������� �������
  wait_event(!(I2C_SR1_TXE && I2C_SR1_BTF), 1);
  
  //��������� �����-������� (�������)
  I2C_CR2_START = 1;
  //���� ��������� ���� SB
  wait_event(!I2C_SR1_SB, 1);
  
  //���������� � ������� ������ ����� �������� ���������� � ���������
  //� ����� ������ (���������� �������� ���� � 1)
  I2C_DR = address | 0x01;
  
  //������ �������� ������� �� ���������� ����������� ����
  //N=1
  if(length == 1){
    //��������� ������������� � ����� �������
    I2C_CR2_ACK = 0;
    //���� ������������� �������� ������
    wait_event(!I2C_SR1_ADDR, 1);
    
    //�������� �� Errata
    __disable_interrupt();
    //������� ���� ADDR ������� �������� SR3
    I2C_SR3;
    
    //����������� ��� STOP
    I2C_CR2_STOP = 1;
    //�������� �� Errata
    __enable_interrupt();
    
    //���� ������� ������ � RD
    wait_event(!I2C_SR1_RXNE, 1);
    
    //������ �������� ����
    *data = I2C_DR;
  } 
  //N=2
  else if(length == 2){
    //��� ������� ��������� NACK �� ��������� �������� �����
    I2C_CR2_POS = 1;
    //���� ������������� �������� ������
    wait_event(!I2C_SR1_ADDR, 1);
    //�������� �� Errata
    __disable_interrupt();
    //������� ���� ADDR ������� �������� SR3
    I2C_SR3;
    //��������� ������������� � ����� �������
    I2C_CR2_ACK = 0;
    //�������� �� Errata
    __enable_interrupt();
    //���� �������, ����� ������ ���� �������� � DR,
    //� ������ � ��������� ��������
    wait_event(!I2C_SR1_BTF, 1);
    
    //�������� �� Errata
    __disable_interrupt();
    //����������� ��� STOP
    I2C_CR2_STOP = 1;
    //������ �������� �����
    *data++ = I2C_DR;
    //�������� �� Errata
    __enable_interrupt();
    *data = I2C_DR;
  } 
  //N>2
  else if(length > 2){
    //���� ������������� �������� ������
    wait_event(!I2C_SR1_ADDR, 1);
    
    //�������� �� Errata
    __disable_interrupt();
    
    //������� ���� ADDR ������� �������� SR3
    I2C_SR3;
    
    //�������� �� Errata
    __enable_interrupt();
    
    while(length-- > 3 && tmo){
      //������� ��������� ������ � DR � ��������� ��������
      wait_event(!I2C_SR1_BTF, 1);
      //������ �������� ���� �� DR
      *data++ = I2C_DR;
    }
    //����� �������� �����
    if(!tmo) return I2C_TIMEOUT;
    
    //�������� ������� 3 ��������� �����
    //����, ����� � DR �������� N-2 ����, � � ��������� ��������
    //�������� N-1 ����
    wait_event(!I2C_SR1_BTF, 1);
    //��������� ������������� � ����� �������
    I2C_CR2_ACK = 0;
    //�������� �� Errata
    __disable_interrupt();
    //������ N-2 ���� �� RD, ��� ����� �������� ������� � ���������
    //������� ���� N, �� ������ � ����� ������ ���������� ������� NACK
    *data++ = I2C_DR;
    //������� STOP
    I2C_CR2_STOP = 1;
    //������ N-1 ����
    *data++ = I2C_DR;
    //�������� �� Errata
    __enable_interrupt();
    //����, ����� N-� ���� ������� � DR �� ���������� ��������
    wait_event(!I2C_SR1_RXNE, 1);
    //������ N ����
    *data++ = I2C_DR;
  }
  
  //���� �������� ���� �������
  wait_event(I2C_CR2_STOP, 1);
  //���������� ��� POS, ���� ����� �� ��� ����������
  I2C_CR2_POS = 0;
  
  return I2C_SUCCESS;
}
