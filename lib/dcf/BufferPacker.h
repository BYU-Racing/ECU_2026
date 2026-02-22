#ifndef BUFFERPACKER_H
#define BUFFERPACKER_H

#include <cstdint>
#include <cstddef>
#include <cstring>

/**
 * <b>Helper class for packing and unpacking objects into a bytes buffer.</b>
 *
 * The SIZE template parameter is used to determine the internal buffer size and <b>MUST</b>
 * be known at compile time, since it is <i>not</i> dynamically allocated.
 *
 * Because of this, SIZE must be either:
 *
 * - constexpr — preferred for type safety
 * <code>
 * constexpr size_t BUFFER_SIZE = 6;
 * BufferPacker packer = BufferPacker<BUFFER_SIZE>();
 * </code>
 * - macro defined — preferred for size optimization
 * <code>
 * #define BUFFER_SIZE = 6;
 * BufferPacker packer = BufferPacker<BUFFER_SIZE>();
 * </code>
 * - rvalue / magic number — <b>not</b> preferred
 * <code>
 * BufferPacker packer = BufferPacker<6>();
 * </code>
 * @tparam SIZE The size of the internal, stack-allocated, fixed-size bytes buffer; defaults to 8.
 */
template <size_t SIZE = 8> class BufferPacker
{
public:
    /**
     * A BufferPacker constructed without a source buffer is put into PACK mode and has its internal buffer initialized
     * to all 0s.
     */
    BufferPacker() : m_DataSize(SIZE), m_Offset(0), m_Mode(PACK)
    {
    }

    /**
     * A BufferPacker constructed with a source buffer is put instantly into FAILURE mode if the size of the source buffer
     * is larger than the size of the internal buffer. Otherwise, the BufferPacker is put into UNPACK mode and his the
     * data in the src buffer copied to the internal buffer.
     */
    explicit BufferPacker(const uint8_t* src, const size_t srcSize) : m_DataSize(srcSize), m_Offset(0)
    {
        if (srcSize > SIZE)
        {
            m_Mode = FAILURE;
        } else
        {
            m_Mode = UNPACK;
            memcpy(m_Buffer, src, srcSize);
        }
    }

    /**
     * A BufferPacker constructed with a source buffer is put instantly into FAILURE mode if the size of the source buffer
     * is larger than the size of the internal buffer. Otherwise, the BufferPacker is put into UNPACK mode and his the
     * data in the src buffer copied to the internal buffer.
     */
    template <size_t SRC_SIZE> explicit BufferPacker(const uint8_t (&src)[SRC_SIZE]) : m_DataSize(SRC_SIZE), m_Offset(0)
    {
        if (SRC_SIZE > SIZE)
        {
            // Buffer overflow - set failure mode
            m_Mode = FAILURE;
        } else
        {
            m_Mode = UNPACK;
            memcpy(m_Buffer, src, SRC_SIZE);
        }
    }

    // Delete copy and move constructors/operators

    BufferPacker(const BufferPacker&) = delete;
    BufferPacker& operator=(const BufferPacker&) = delete;
    BufferPacker(BufferPacker&&) = delete;
    BufferPacker& operator=(BufferPacker&&) = delete;

    /** This conversion returns false if a BufferPacker has "failed", true otherwise. */
    explicit operator bool() const
    {
        return m_Mode != FAILURE;
    }

    /** Thisreturns false if a BufferPacker has "failed", true otherwise. */
    [[nodiscard]] bool hasFailed() const
    {
        return m_Mode != FAILURE;
    }

    /**
     * <b>Pack any type of value into the internal bytes buffer.</b>
     *
     * This method can return without packing the value if the BufferPacker either:
     * - fails to pack a value that is larger than the remaining buffer space (buffer overflow)
     * - has failed on a previous call of pack()
     * - is called after being initialized with a source buffer (i.e. in 'UNPACK' mode)
     *
     * @tparam T any type that can be copied safely with c-style memcpy
     * @param value the object to copy into the buffer
     */
    template <typename T> void pack(T value)
    {
        if (m_Mode != PACK)
        {
            return;
        }
        if (m_Offset + sizeof(T) > SIZE)
        {
            // Buffer overflow - set FAILURE mode
            m_Mode = FAILURE;
            return;
        }
        memcpy(&m_Buffer[m_Offset], &value, sizeof(T));
        m_Offset += sizeof(T);
        if (m_Offset > m_DataSize)
        {
            m_DataSize = m_Offset; // Keep counter of actual data size accurate
        }
    }

    /**
     * <b>Unpack the value of any type from the internal bytes buffer.</b>
     *
     * This method can return an uninitialized T early if the BufferPacker either:
     * - fails to unpack a value that is larger than the remaining buffer space (buffer overread)
     * - has failed on a previous call of unpack()
     * - is called after being initialized without a source buffer (i.e. in 'PACK' mode)
     *
     * @tparam T any type that can by copied safely with c-style memcpy
     * @return The value unpacked from the buffer; could be uninitialized if a failure occured
     */
    template <typename T> T unpack()
    {
        T value{};
        if (m_Mode != UNPACK)
        {
            return value;
        }
        if (m_Offset + sizeof(T) > m_DataSize)
        {
            // Buffer overread - set FAILURE mode
            m_Mode = FAILURE;
            return value;
        }
        memcpy(&value, &m_Buffer[m_Offset], sizeof(T));
        m_Offset += sizeof(T);
        return value;
    }

    /**
     * <b>Skip over value of any type from the internal bytes buffer.</b>
     *
     * Like unpack(), but only moves to the next item without returning a value.
     *
     * This method can return early if the BufferPacker either:
     * - fails to skip over a value that is larger than the remaining buffer space (buffer overread)
     * - has failed on a previous call of skip()
     * - is called after being initialized without a source buffer (i.e. in 'PACK' mode)
     *
     * @tparam T any type
     */
    template <typename T> void skip()
    {
        if (m_Mode != UNPACK)
        {
            return;
        }
        if (m_Offset + sizeof(T) > m_DataSize)
        {
            // Buffer overread - set FAILURE mode
            m_Mode = FAILURE;
            return;
        }
        m_Offset += sizeof(T);
    }

    /**
     * <b>Seek the value of any type from the internal bytes buffer.</b>
     *
     * Like unpack(), but only returns a value without moving to the next item.
     *
     * This method can return an uninitialized T early if the BufferPacker either:
     * - fails to seek a value that is larger than the remaining buffer space (buffer overread)
     * - has failed on a previous call of seek()
     * - is called after being initialized without a source buffer (i.e. in 'PACK' mode)
     *
     * @tparam T any type that can by copied safely with c-style memcpy
     * @return The value unpacked from the buffer; could be uninitialized if a failure occured
     */
    template <typename T> T seek()
    {
        T value{};
        if (m_Mode != UNPACK)
        {
            return value;
        }
        if (m_Offset + sizeof(T) > m_DataSize)
        {
            // Buffer overread - set FAILURE mode
            m_Mode = FAILURE;
            return value;
        }
        memcpy(&value, &m_Buffer[m_Offset], sizeof(T));
        return value;
    }

    /**
     * <b>Deep copy the values of the internal buffer into another fixed-size, external buffer.</b>
     *
     * This method can return early if the BufferPacker either:
     * - fails to deep copy the internal buffer if it is larger than the destination buffer space (buffer overflow)
     * - has failed on a previous call of pack() or unpack()
     *
     * @tparam DEST_SIZE the size of a fixed-size buffer
     * @param dest the external buffer to copy the internal buffer's data to
     */
    template <size_t DEST_SIZE> void deepCopyTo(uint8_t (&dest)[DEST_SIZE])
    {
        if (m_Mode == FAILURE)
        {
            return;
        }
        if (m_DataSize > DEST_SIZE)
        {
            // Buffer overflow - set failure mode
            m_Mode = FAILURE;
            return;
        }
        memcpy(dest, m_Buffer, m_Offset);
    }

    /** @return the number of bytes in the internal buffer */
    [[nodiscard]] size_t getBufferSize() const
    {
        return m_DataSize;
    }

    /**
     * <b>Reset the internal buffer to 0 and enter 'PACK' Mode</b>
     *
     * @param clearBuffer flag for whether to 0 out the memory in the internal buffer - can be slow
     */
    void reset(const bool clearBuffer=false)
    {
        m_Offset = 0;
        m_DataSize = 0;
        m_Mode = PACK;
        if (clearBuffer)
        {
            memset(m_Buffer, 0, SIZE);
        }
    }

    /**
     * <b>Reset the internal buffer with a new src and enter 'UNPACK' Mode</b>
     *
     * This method will instantly enter failure mode if
     * the size of the src buffer is larger than the size of the internal buffer.
     */
    template <size_t SRC_SIZE> void reset(const uint8_t (&src)[SRC_SIZE])
    {
        if (SRC_SIZE > SIZE)
        {
            m_Mode = FAILURE;
            return;
        }
        m_Offset = 0;
        m_DataSize = SRC_SIZE;
        m_Mode = UNPACK;
        memset(m_Buffer, 0, SIZE);
        memcpy(m_Buffer, src, SRC_SIZE);
    }

    /**
     *  <b>Creates a new heap-allocated copy of the internal buffer that the caller owns.</b>
     *
     * @return A heap-allocated array that the caller owns; can be nullptr if BufferPacker is in failure mode.
     */
    [[nodiscard]] uint8_t* getOwnedHeapBuffer() const
    {
        if (m_Mode == FAILURE)
        {
            return nullptr;
        }
        uint8_t* heapBuffer = new uint8_t[m_DataSize];
        memcpy(heapBuffer, m_Buffer, m_DataSize);
        return heapBuffer;
    }
private:
    /** Modes the BufferPacker can run in */
    enum Mode : uint8_t {
        /** Mode that enables pack() and disable unpack() */
        PACK,
        /** Mode that enables unpack() and disable pack() */
        UNPACK,
        /** Mode that disables pack() and unpack() */
        FAILURE,
    };
    /** Internal, stack allocated, fixed-size buffer for packing. Size is determined at compile time by SIZE tparam. */
    uint8_t m_Buffer[SIZE] = {};
    /** Amount of byte in the internal buffer. */
    size_t m_DataSize;
    /** Byte position counter that determines where in the m_Buffer the next operation will begin. */
    size_t m_Offset;
    /** Mode to track what operations can occur, and whether a failure has happened. */
    Mode m_Mode;
};

#endif //BUFFERPACKER_H
