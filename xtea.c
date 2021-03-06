
// xtea.c
// https://ru.wikipedia.org/wiki/XTEA

/*    ��������� ��������� ������ ���������� XTEA
  ������ �������� (XTEA-1) ����� ���������� �������� � ��� ����������� ���������� 
  ������� (�� 32 �� 64) �������� ������� �����������. XTEA-2 ������������ ����� 
  ���������� � ������� �������� �����, � �� �� ����� ������� ��� XTEA-1. 
  XTEA-3 � ��� ���������� �������� � �������������� �������� ������� ����� � 
  �����. ������ ������� �������� ������� ���������, �� ����� �������. ��� ��� 
  ��� ��������� ��������� �� ���� ������������� TEA � ����������� ���� ��������� 
  �����������, �� �� ����� ������� ���������� ���������.
  ������������� ������� ����������:
      �������� | ���. ���-�� | ����. ���-�� | ������  |  ������ 
               |   �������	 |    �������	  |  �����  |  �����
      -----------------------------------------------------------
        XTEA-1 |     32	     |      64	    | 64 ����	|  128 ���
        XTEA-2 |     64	     |     128	    | 128 ���	|  128 ���
        XTEA-3 |     64	     |     128	    | 128 ���	|  256 ���

  ��������� ����� ���� ����������. ������ �������� ������� � ���, ������ ������� 
  ���� ��������� ������ ������������ ��� ������������ �������� ������ ����� 
  (��� � XTEA-1 � XTEA-2). ������ �������� ����������� � ��� ��� ���� �������� 
  �� ��� ��������� �� 4 ��������, � ������ ����� ��������� ���������� ������ 
  ���� ��������� (��� � XTEA-3). XTEA-3 ����� ���� �������� ���� ������������� 
  ���� ������ ��������� � ����� ������ ���������.
*/

#include "xtea.h"


// *****************************************************************************
// ************** � � � � � � � � * X T E A - 1 ********************************
// ���� �� ����������� XTEA ������� � ���, ��� ���� � ����� ������ �� ���� � �� 
// �� ���� � ������ ������ ���������. ��� �������� ����� ���� ��������� ���� 
// ������������� �����, ����������� � ���� �������� ���������� ������. 
// ������������ ������ ������������ ����������� � �� ������� ��������� ������. 
// ���������� ������ �������������� ���� ������������ �������� ������ ����� �� 
// ��������, ��������� �� ��������� ������. �������� XTEA-1 ��������� ��� ���� 
// �� �������� ����� XTEA ���� ��������������� ��������� ��������� ����� ��� 
// ��������� �������� ��������� ���������.
// ���� ���������� ���������� ����������� � ��������� �� ������ ������������� 
// ��������, ��� ���������� ������������, ��������� �������� �������� �������� 
// ���������� � ����������� ������������ ��� ����� ���������� �� ��������������� 
// �� ����� �����������.

// *****************************************************************************
// ��������� ����������� ������� ����� �����, ��������� 5 ������� ����� 
// ���������� shift. ���� �������� ���������� ������ ���� ����� �� �����, ��� 
// �������� ���������� � ������ �������� �� ����������� ����������� �����������
static inline uint32_t rol(uint32_t base, uint32_t shift)
{
  // only 5 bits of shift are significant
  shift &= 0x1F;
  uint32_t res = (base << shift) | (base >> (32 - shift));
  return res;
}


// *****************************************************************************
// ����������� �� ��������� XTEA-1
// [num_rounds] - ���-�� ������� (32 - 64)
// [*pData]     - ��������� �� ������ (2 �������� �� 32 ����), ��������� ���
// [*pKey]      - ��������� �� ���� ���������� (4 �������� �� 32 ����)
void xtea1_encipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey)
{
  uint32_t delta = 0x9E3779B9;
	uint32_t sum = 0;
	
  // load and pre-white the registers
	uint32_t y = pData[0] + pKey[0];
	uint32_t z = pData[1] + pKey[1];
	
  // Round functions
	for(uint32_t i = 0; i < num_rounds; i++) 
	{
		y += ((z << 4) ^ (z >> 5)) + (z ^ sum) + rol(pKey[sum & 3], z);
		sum += delta;
		z += ((y << 4) ^ (y >> 5)) + (y ^ sum) + rol(pKey[(sum >> 11) & 3], y);
	}
  
	// post-white and store registers
	pData[0] = y ^ pKey[2];
	pData[1] = z ^ pKey[3];
  return;
}


// *****************************************************************************
// ������������ �� ��������� XTEA-1
// [num_rounds] - ���-�� ������� (32 - 64)
// [*pData]     - ��������� �� ������ (2 �������� �� 32 ����), ��������� ���
// [*pKey]      - ��������� �� ���� ���������� (4 �������� �� 32 ����)
void xtea1_decipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey)
{
	uint32_t delta = 0x9E3779B9;
  uint32_t sum = delta * num_rounds;
	
  uint32_t z = pData[1] ^ pKey[3];
	uint32_t y = pData[0] ^ pKey[2];
  
	for(uint32_t i = 0; i < num_rounds; i++) 
	{
		z -= ((y << 4) ^ (y >> 5)) + (y ^ sum) + rol(pKey[(sum >> 11) & 3], y);
		sum -= delta;
		y -= ((z << 4) ^ (z >> 5)) + (z ^ sum) + rol(pKey[sum & 3], z);
	}
	
  pData[1] = z - pKey[1];
	pData[0] = y - pKey[0];
  return;
}


// *****************************************************************************
// ************** � � � � � � � � * X T E A - 2 ********************************
// ���������� XTEA-1 � ������������� 128 ������� �����. ���������� �������� 
// ������� ������ �������, �� �������� ���������� � ���� ���� ��� � XTEA.
// �������� ������������ ����� ��������� � ��� ����������� ��������� ������� 
// �����. ���� �������� ���������� �� �� ������� �������� ��� � XTEA-1, �� 
// ������� ������ ��������. �� ����� ���� �� ������� � ��� ���� ������ �������� 
// �� 32 �� 64 (�� 64 �� 128 �������). 48 �������� � ��� ���������� ����� 
// ��������� � ����������� ����������.
// *****************************************************************************

// *****************************************************************************
// ����������� �� ��������� XTEA-2
// [num_rounds] - ���-�� ������� (64 - 128)
// [*pData]     - ��������� �� ������ (4 �������� �� 32 ����), ��������� ���
// [*pKey]      - ��������� �� ���� ���������� (4 �������� �� 32 ����)
void xtea2_encipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey)
{
  uint32_t t;
  uint32_t delta = 0x9E3779B9;
	uint32_t sum = 0;
  
	uint32_t a = pData[0];
	uint32_t b = pData[1] + pKey[0];
	uint32_t c = pData[2];
	uint32_t d = pData[3] + pKey[1];
  
	for(uint32_t i = 0; i < num_rounds; i++) 
  {
		a += ((b << 4) ^ (b >> 5)) + (d ^ sum) + rol(pKey[sum & 3], b);
		sum += delta;
		c += ((d << 4) ^ (d >> 5)) + (b ^ sum) + rol(pKey[(sum >> 11) & 3], d);
		t = a; 
    a = b; 
    b = c; 
    c = d; 
    d = t;
	}
  
	pData[0] = a ^ pKey[2];
	pData[1] = b;
	pData[2] = c ^ pKey[3];
	pData[3] = d;
  return;
}


// *****************************************************************************
// ������������ �� ��������� XTEA-2
// [num_rounds] - ���-�� ������� (64 - 128)
// [*pData]     - ��������� �� ������ (4 �������� �� 32 ����), ��������� ���
// [*pKey]      - ��������� �� ���� ���������� (4 �������� �� 32 ����)
void xtea2_decipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey)
{
	uint32_t t;
  uint32_t delta = 0x9E3779B9;
  uint32_t sum = delta * num_rounds;
  
	uint32_t d = pData[3];
	uint32_t c = pData[2] ^ pKey[3];
	uint32_t b = pData[1];
	uint32_t a = pData[0] ^ pKey[2];
  
	for(uint32_t i = 0; i < num_rounds; i++) 
  {
		t = d; 
    d = c; 
    c = b; 
    b = a; 
    a = t;
		c -= ((d << 4) ^ (d >> 5)) + (b ^ sum) + rol(pKey[(sum >> 11) & 3], d);
		sum -= delta;
		a -= ((b << 4) ^ (b >> 5)) + (d ^ sum) + rol(pKey[sum & 3], b);
	}
  
	pData[0] = a;
	pData[1] = b - pKey[0];
	pData[2] = c;
	pData[3] = d - pKey[1];
  return;
}


// *****************************************************************************
// ************** � � � � � � � � * X T E A - 3 ********************************
// ���������� XTEA-1 � ������������� 256 ������� ����� � ����� ����������� 128 
// ������� �����. ���� �������� ������� �� 32 �� 64 ��������, �� � �� �� ����� 
// ������������ �������� ������ �� ���� ���� ������� ��������. ���� ���������� 
// ���������� ����������� � ��������� ��������, ��������� �� �����, ��� 
// ���������� ������������.
// XTEA-3 ���������� 5 ������� � 5 ������� ��� �������� ��������� ������ ��� 
// ������������ ������ �����, ������ ��� �������������� ������ ������� � ���, 
// ��� ��� ���� �������� ���������� ����������. ���� �������� ��� �� ������� ��� 
// ������� 32 ��������, ������, 48 �������� � ��� ������������� ����������� 
// ����� ��������� � ����������� ���������� ������.
// *****************************************************************************

// *****************************************************************************
// ����������� �� ��������� XTEA-3
// [num_rounds] - ���-�� ������� (64 - 128)
// [*pData]     - ��������� �� ������ (4 �������� �� 32 ����), ��������� ���
// [*pKey]      - ��������� �� ���� ���������� (8 �������� �� 32 ����)
void xtea3_encipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey)
{
	uint32_t t;
  uint32_t delta = 0x9E3779B9;
	uint32_t sum = 0;
  
	uint32_t a = pData[0] + pKey[0];
	uint32_t b = pData[1] + pKey[1];
	uint32_t c = pData[2] + pKey[2];
	uint32_t d = pData[3] + pKey[3];
  
	for(uint32_t i = 0; i < num_rounds; i++)
  {
		a += (((b << 4) + rol (pKey[(sum % 4) + 4], b)) ^
          (d + sum) ^ ((b >> 5) + rol (pKey[sum % 4], b >> 27)));
		sum += delta;
    c += (((d << 4) + rol (pKey[((sum >> 11) % 4) + 4], d)) ^
          (b + sum) ^ ((d >> 5) + rol (pKey[(sum >> 11) % 4], d >> 27)));
    t = a;
    a = b;
    b = c;
    c = d;
    d = t;
	}
  
	pData[0] = a ^ pKey[4];
	pData[1] = b ^ pKey[5];
	pData[2] = c ^ pKey[6];
	pData[3] = d ^ pKey[7];
  return;
}


// *****************************************************************************
// ������������ �� ��������� XTEA-3
// [num_rounds] - ���-�� ������� (64 - 128)
// [*pData]     - ��������� �� ������ (4 �������� �� 32 ����), ��������� ���
// [*pKey]      - ��������� �� ���� ���������� (8 �������� �� 32 ����)
void xtea3_decipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey)
{
  uint32_t t;
  uint32_t delta = 0x9E3779B9;
  uint32_t sum = delta * num_rounds;
  
	uint32_t d = pData[3] ^ pKey[7];
	uint32_t c = pData[2] ^ pKey[6];
	uint32_t b = pData[1] ^ pKey[5];
	uint32_t a = pData[0] ^ pKey[4];
	
  for(uint32_t i = 0; i < num_rounds; i++)
  {
		t = d;
    d = c;
    c = b;
    b = a;
    a = t;
		c -= (((d << 4) + rol (pKey[((sum >> 11) % 4) + 4], d)) ^
          (b + sum) ^ ((d >> 5) + rol (pKey[(sum >> 11) % 4], d >> 27)));
    sum -= delta;
		a -= (((b << 4) + rol (pKey[(sum % 4) + 4], b)) ^
          (d + sum) ^ ((b >> 5) + rol (pKey[sum % 4], b >> 27)));
  }
  
	pData[3] = d - pKey[3];
	pData[2] = c - pKey[2];
	pData[1] = b - pKey[1];
	pData[0] = a - pKey[0];

  return;
}


// *****************************************************************************
// ���� ��������� XTEA-3
static void xtea3_test(void)
{
  const uint32_t key[8] = { 0x583a2d21, 0x693a4172, 0x5d493e4f, 0x45787122,
                            0x36363b54, 0x5b7d773c, 0x6e647236, 0x3448343d };

  uint8_t data[121];
  uint8_t data_res[(sizeof(data) / (4 * 4) + 1) * (4 * 4)];
  
  // ������ ������� ������
  for(uint32_t i = 0; i < sizeof(data); i++)
    data[i] = 0x20 + i;
 
  // * ����������� *
  // ������� ���-�� ���� ��� ����������� (= ������� �������)
  uint32_t cnt = sizeof(data);
  // ��������� �� ������ ������, ��������� ����� ������������ (�� ������� 
  // �������� XTEA3_DATA_MIN)
  uint8_t* pData = data;
  // ��������� �� ������ ���������� �������������� ������, (������� �������� 
  // XTEA3_DATA_MIN)
  uint8_t* pData_res = data_res;
  // ��������� �� ������ ������ ��� �����������
  uint32_t* pData_res_start = (uint32_t*)pData_res;
  // ���� ����������� ������, ���� ���� ������
  while(cnt > XTEA3_DATA_MIN)
  {
    // ����������� ������ ��� ���������� 1
    PTR_PVAL_WRITE_PVAL_INC2_type(uint32_t, pData_res, pData);
    // ����������� ������ ��� ���������� 2
    PTR_PVAL_WRITE_PVAL_INC2_type(uint32_t, pData_res, pData);
    // ����������� ������ ��� ���������� 2
    PTR_PVAL_WRITE_PVAL_INC2_type(uint32_t, pData_res, pData);
    // ����������� ������ ��� ���������� 2
    PTR_PVAL_WRITE_PVAL_INC2_type(uint32_t, pData_res, pData);
    // ����������� �� ��������� XTEA-3
    xtea3_encipher((XTEA3_ROUND_MAX + XTEA3_ROUND_MIN) / 2, pData_res_start, key);
    // ��������� ������� ������������ ����
    cnt -= XTEA3_DATA_MIN;
    // ������� �� ��������� ������ ������ ��� ����������
    pData_res_start = (uint32_t*)pData_res;
  }

  // �������� ������� ���-�� ���� ��� ����������� 
  if(cnt != 0)
  { // ���� != 0, �� ������ �� ��������� �� ������� XTEA3_DATA_MIN
    // ���� ����������� ���������� �� ���������� ������
    for(uint32_t i = sizeof(data) - cnt; i < sizeof(data); i++)
      *pData_res++ = *pData++;
    // ���� ����������� ������ ���������� �� ����������� �������
    for(uint32_t i = cnt; i < XTEA3_DATA_MIN; i++)
      *pData_res++ = 0xFF;
    // ����������� �� ��������� XTEA-3 (������� ������)
    xtea3_encipher((XTEA3_ROUND_MAX + XTEA3_ROUND_MIN) / 2, pData_res_start, key);
  }
  
  // * ������������ *
  // ��������� �� ������ �����������
  uint32_t* pData_res1 = (uint32_t*)data_res;
  // C������ ��������������� ����
  cnt = 0;
  // ���� �������������� ����� �������
  while(cnt < sizeof(data_res))
  {
    // ������������ �� ��������� XTEA-3
    xtea3_decipher((XTEA3_ROUND_MAX + XTEA3_ROUND_MIN) / 2, pData_res1, key);
    // ���������������� ��������� �� ������ ��� ���������� ������������
    pData_res1 += XTEA3_PTR_DATA_INC;
    // ��. ������������ ������� ����
    cnt += XTEA3_DATA_MIN;
  }
  return;
}