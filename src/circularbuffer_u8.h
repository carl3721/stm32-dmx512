#ifndef CIRCULARBUFFER_U8_H
#define	CIRCULARBUFFER_U8_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

    /**
     * @brief data structure for the circular byte buffer.
     * 
     * The elements are marked as volatile because this buffer may be used in
     * interrupt handlers.
     */
    typedef struct {
        /**
         * @brief The size of the buffer.
         */
        volatile uint32_t size;
        /**
         * @brief The start or read position of the buffer.
         */
        volatile uint32_t start;
        /**
         * @brief The amount of elements in the buffer.
         */
        volatile uint32_t count;
        /**
         * @brief The elements in the buffer.
         */
        volatile uint8_t* elements;
    } circularbuffer_u8_s;

    /**
     * @brief Initializes the circular buffer.
     * 
     * It is always best to take a power of 2 for the size. If a power of 2 is 
     * taken, the compiler will optimize the modulo operators in the code to
     * simple bit-shift operations which are a lot faster in execution.
     * 
     * @param cb Pointer to the circular buffer to initialize.
     * @param size The size of the backing element array.
     * @param elements The backing element array.
     */
    void circularbuffer_u8_init(circularbuffer_u8_s* cb, uint32_t size,
            volatile uint8_t* elements);

    /**
     * @brief Checker method to see if there is free space in the circular 
     * buffer.
     * 
     * @param cb The circular buffer to check.
     * 
     * @return <code>1</code> if the buffer is full, <code>0</code> if not.
     */
    uint8_t circularbuffer_u8_full(circularbuffer_u8_s* cb);

    /**
     * @brief Cehcker method to see if there are more elements in the circular
     * buffer.
     * 
     * @param cb The circular buffer to check.
     * 
     * @return  <code>1</code> if the buffer is empty, <code>0</code> if not.
     */
    uint8_t circularbuffer_u8_empty(circularbuffer_u8_s* cb);

    /**
     * @brief Pushes an element at the end of the circular buffer.
     * 
     * @param cb The circular buffer to put the element into.
     * @param byte The element to put in the circular buffer.
     */
    void circularbuffer_u8_push(circularbuffer_u8_s* cb, uint8_t byte);

    /**
     * @brief Pops an element from the head of the circular buffer.
     * 
     * @param cb The circular buffer to take the element from.
     * 
     * @return The element taken from the buffer.
     */
    uint8_t circularbuffer_u8_pop(circularbuffer_u8_s* cb);

#ifdef	__cplusplus
}
#endif

#endif	/* CIRCULARBUFFER_U8_H */

