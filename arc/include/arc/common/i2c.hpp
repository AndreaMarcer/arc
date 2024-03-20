/**
 * @file i2c.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include "hardware/i2c.h"

/*****************************************************************************\
|                                    COMMON                                   |
\*****************************************************************************/
namespace arc::common {

struct I2C {
    I2C(uint8_t dev_addr) : m_dev_addr{dev_addr}, m_i2c_inst{i2c_default} {};
    I2C(uint8_t dev_addr, i2c_inst_t *i2c_inst)
        : m_dev_addr{dev_addr}, m_i2c_inst{i2c_inst} {};

    /**
     * @brief I2C address of the device.
     */
    uint8_t m_dev_addr;

    /**
     * @brief I2C PICO instance.
     */
    i2c_inst_t *m_i2c_inst;

    /**
     * @brief Writes multiple bytes to a register.
     *
     * @param data Bytes to write. Generally the first Byte is the register
     * address to write to
     * @param len Number of bytes to write.
     * @return int Number of bytes written, or PICO_ERROR_GENERIC if address not
     * acknowledged, no device present.
     */
    inline int writeBytes(uint8_t *data, const uint8_t len = 1) {
        return i2c_write_blocking(m_i2c_inst, m_dev_addr, data, len, false);
    }

    /**
     * @brief
     *
     * @param addr
     * @param data
     * @param mask
     * @return int
     */
    inline int writeBits(const uint8_t addr, const uint8_t data,
                         const uint8_t mask) {
        int ret = 0;

        uint8_t curr_byte;
        ret = readBytes(addr, &curr_byte);
        if (ret != 1) {
            log_error << "Error in readBytes().n";
            return 0;
        }

        uint8_t new_byte = curr_byte;
        if (data) {  // set bit to 1
            new_byte |= mask;
        } else {  // set bit to 0
            new_byte &= ~mask;
        }

        uint8_t buf[] = {addr, new_byte};
        ret = writeBytes(buf, 2);
        if (ret != 2) {
            log_error << "Error in writeBytes()n";
            return 0;
        }
        return 1;
    }

    /**
     * @brief Set the Bits object
     *
     * @param addr
     * @param mask
     * @return int
     */
    inline int setBits(const uint8_t addr, const uint8_t mask) {
        return writeBits(addr, 1, mask);
    }

    /**
     * @brief
     *
     * @param addr
     * @param mask
     * @return int
     */
    inline int unsetBits(const uint8_t addr, const uint8_t mask) {
        return writeBits(addr, 0, mask);
    }

    /**
     * @brief
     *
     * @param addr
     * @param data
     * @param mask
     * @param offset
     * @return int
     */
    inline int writeBits(const uint8_t addr, const uint8_t data,
                         const uint8_t mask, const uint8_t offset) {
        int ret = 0;

        uint8_t curr_byte;
        ret = readBytes(addr, &curr_byte);
        if (ret != 1) {
            log_error << "Error in readBytes().n";
            return 0;
        }

        uint8_t new_byte = curr_byte & ~mask;  // clear bits
        new_byte |= (static_cast<uint8_t>(data) << offset) & mask;

        uint8_t buf[] = {addr, new_byte};
        ret = i2c_write_blocking(m_i2c_inst, m_dev_addr, buf, 2, false);
        if (ret != 2) {
            log_error << "Error in writeBytes()n";
            return 0;
        }
        return 1;
    }

    /**
     * @brief
     *
     * @param addr
     * @param data
     * @param len
     * @return int
     */
    inline int writeWords(const uint8_t addr, uint16_t *data,
                          const uint8_t len = 1) {
        uint8_t num_bytes = len * 2 + 1;
        uint8_t buf[num_bytes];

        buf[0] = addr;
        for (int i = 0; i < len; i++) {
            buf[i * 2 + 1] = data[i] >> 8;
            buf[i * 2 + 2] = data[i];
        }

        int ret = writeBytes(buf, num_bytes);
        if (ret != num_bytes) {
            log_error << "Error in writeBytes()\n";
            return EIO;
        }
        return (num_bytes - 1) / 2;
    }

    /**
     * @brief
     *
     * @param addr
     * @param mask
     * @param offset
     * @param value
     * @return int
     */
    inline int readBits(const uint8_t addr, const uint8_t mask,
                        const uint8_t offset, uint8_t &value) {
        int ret = 0;

        uint8_t byte;
        ret = readBytes(addr, &byte);
        if (ret != 1) {
            log_error << "Error in readBytes().n";
            return 0;
        }

        value = (byte & mask) >> offset;  // clear bits
        return 1;
    }

    /**
     * @brief Read multiple bytes from a register.
     *
     * @param reg_addr The register address to start reading from.
     * @param len Number of bytes to read.
     * @param buf Buffer to write to.
     * @return int Number of bytes read, or PICO_ERROR_GENERIC if address not
     * acknowledged or no device present.
     */
    inline int readBytes(const uint8_t reg_addr, uint8_t *buf,
                         const uint8_t len = 1) {
        i2c_write_blocking(m_i2c_inst, m_dev_addr, &reg_addr, 1, true);
        return i2c_read_blocking(m_i2c_inst, m_dev_addr, buf, len, false);
    }

    inline int readWords(const uint8_t reg_addr, uint16_t *buf,
                         const uint8_t len = 1) {
        i2c_write_blocking(m_i2c_inst, m_dev_addr, &reg_addr, 1, true);

        uint8_t num_bytes = len * 2;
        uint8_t buf_bytes[num_bytes];

        int ret = i2c_read_blocking(m_i2c_inst, m_dev_addr, buf_bytes,
                                    num_bytes, false);
        if (ret != num_bytes) {
            log_error << "Error in readBytes().\n";
            return 0;
        }

        for (uint8_t i = 0; i < len; i++) {
            buf[i] = (buf_bytes[i * 2] << 8) | (buf_bytes[i * 2 + 1]);
        }

        return ret / 2;
    }
};

}  // namespace arc::common