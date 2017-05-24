
// xtea.h

/*  Сравнительная таблица алгоритмов:
      Название | Мин. кол-во | Макс. кол-во | Размер  |  Размер 
               |   раундов	 |    раундов	  |  блока  |  ключа
      -----------------------------------------------------------
        XTEA-1 |     32	     |      64	    | 64 бита	|  128 бит
        XTEA-2 |     64	     |     128	    | 128 бит	|  128 бит
        XTEA-3 |     64	     |     128	    | 128 бит	|  256 бит
*/

#ifndef XTEA_H
#define XTEA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Кол-во раундов для XTEA
#define XTEA1_ROUND_MIN               (32)
#define XTEA1_ROUND_MAX               (64)
#define XTEA2_ROUND_MIN               (64)
#define XTEA2_ROUND_MAX               (128)
#define XTEA3_ROUND_MIN               (64)
#define XTEA3_ROUND_MAX               (128)

// Значение инкремента указателя
#define XTEA1_PTR_DATA_INC            (2)
#define XTEA2_PTR_DATA_INC            (4)
#define XTEA3_PTR_DATA_INC            (4)

// Мин. Кол-во данных в байтах для шиврования XTEA
#define XTEA1_DATA_MIN                (XTEA1_PTR_DATA_INC * sizeof(uint32_t))
#define XTEA2_DATA_MIN                (XTEA2_PTR_DATA_INC * sizeof(uint32_t))
#define XTEA3_DATA_MIN                (XTEA3_PTR_DATA_INC * sizeof(uint32_t))

// Зашифровать по алгоритму XTEA-1
void xtea1_encipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey);
// Расшифровать по алгоритму XTEA-1
void xtea1_decipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey);

// Зашифровать по алгоритму XTEA-2
void xtea2_encipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey);
// Расшифровать по алгоритму XTEA-2
void xtea2_decipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey);

// Зашифровать по алгоритму XTEA-3
void xtea3_encipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey);
// Расшифровать по алгоритму XTEA-3
void xtea3_decipher(uint32_t num_rounds, uint32_t* pData, uint32_t const* pKey);
// Тест алгоритма XTEA-3
void xtea3_test(void);

#endif  // XTEA_H

