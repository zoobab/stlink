#ifndef STLINK_H
#define STLINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "stlink/defines.h"

    struct stlink_reg {
        uint32_t r[16];
        uint32_t s[32];
        uint32_t xpsr;
        uint32_t main_sp;
        uint32_t process_sp;
        uint32_t rw;
        uint32_t rw2;
        uint8_t control;
        uint8_t faultmask;
        uint8_t basepri;
        uint8_t primask;
        uint32_t fpscr;
    };

    typedef uint32_t stm32_addr_t;

typedef struct flash_loader {
	stm32_addr_t loader_addr; /* loader sram adddr */
	stm32_addr_t buf_addr; /* buffer sram address */
} flash_loader_t;

    typedef struct _cortex_m3_cpuid_ {
        uint16_t implementer_id;
        uint16_t variant;
        uint16_t part;
        uint8_t revision;
    } cortex_m3_cpuid_t;

    typedef struct stlink_version_ {
        uint32_t stlink_v;
        uint32_t jtag_v;
        uint32_t swim_v;
        uint32_t st_vid;
        uint32_t stlink_pid;
    } stlink_version_t;

    typedef struct _stlink stlink_t;

#include "stlink/backend.h"

    struct _stlink {
        struct _stlink_backend *backend;
        void *backend_data;

        // Room for the command header
        unsigned char c_buf[C_BUF_LEN];
        // Data transferred from or to device
        unsigned char q_buf[Q_BUF_LEN];
        int q_len;

        // transport layer verboseness: 0 for no debug info, 10 for lots
        int verbose;
        uint32_t core_id;
        uint32_t chip_id;
        int core_stat;

        char serial[16];
        int serial_size;

        enum stlink_flash_type flash_type;
        stm32_addr_t flash_base;
        size_t flash_size;
        size_t flash_pgsz;

        /* sram settings */
        stm32_addr_t sram_base;
        size_t sram_size;

        // bootloader
        stm32_addr_t sys_base;
        size_t sys_size;

        struct stlink_version_ version;
    };

    int stlink_enter_swd_mode(stlink_t *sl);
    int stlink_enter_jtag_mode(stlink_t *sl);
    int stlink_exit_debug_mode(stlink_t *sl);
    int stlink_exit_dfu_mode(stlink_t *sl);
    void stlink_close(stlink_t *sl);
    int stlink_core_id(stlink_t *sl);
    int stlink_reset(stlink_t *sl);
    int stlink_jtag_reset(stlink_t *sl, int value);
    int stlink_run(stlink_t *sl);
    int stlink_status(stlink_t *sl);
    int stlink_version(stlink_t *sl);
    int stlink_read_debug32(stlink_t *sl, uint32_t addr, uint32_t *data);
    int stlink_read_mem32(stlink_t *sl, uint32_t addr, uint16_t len);
    int stlink_write_debug32(stlink_t *sl, uint32_t addr, uint32_t data);
    int stlink_write_mem32(stlink_t *sl, uint32_t addr, uint16_t len);
    int stlink_write_mem8(stlink_t *sl, uint32_t addr, uint16_t len);
    int stlink_read_all_regs(stlink_t *sl, struct stlink_reg *regp);
    int stlink_read_all_unsupported_regs(stlink_t *sl, struct stlink_reg *regp);
    int stlink_read_reg(stlink_t *sl, int r_idx, struct stlink_reg *regp);
    int stlink_read_unsupported_reg(stlink_t *sl, int r_idx, struct stlink_reg *regp);
    int stlink_write_unsupported_reg(stlink_t *sl, uint32_t value, int r_idx, struct stlink_reg *regp);
    int stlink_write_reg(stlink_t *sl, uint32_t reg, int idx);
    int stlink_step(stlink_t *sl);
    int stlink_current_mode(stlink_t *sl);
    int stlink_force_debug(stlink_t *sl);
    int stlink_target_voltage(stlink_t *sl);
    int stlink_set_swdclk(stlink_t *sl, uint16_t divisor);

    int stlink_erase_flash_mass(stlink_t* sl);
    int stlink_write_flash(stlink_t* sl, stm32_addr_t address, uint8_t* data, uint32_t length, uint8_t eraseonly);
    int stlink_parse_ihex(const char* path, uint8_t erased_pattern, uint8_t * * mem, size_t * size, uint32_t * begin);
    uint8_t stlink_get_erased_pattern(stlink_t *sl);
    int stlink_mwrite_flash(stlink_t *sl, uint8_t* data, uint32_t length, stm32_addr_t addr);
    int stlink_fwrite_flash(stlink_t *sl, const char* path, stm32_addr_t addr);
    int stlink_mwrite_sram(stlink_t *sl, uint8_t* data, uint32_t length, stm32_addr_t addr);
    int stlink_fwrite_sram(stlink_t *sl, const char* path, stm32_addr_t addr);
    int stlink_verify_write_flash(stlink_t *sl, stm32_addr_t address, uint8_t *data, uint32_t length);

    int stlink_chip_id(stlink_t *sl, uint32_t *chip_id);
    int stlink_cpu_id(stlink_t *sl, cortex_m3_cpuid_t *cpuid);

    int stlink_erase_flash_page(stlink_t* sl, stm32_addr_t flashaddr);
    uint32_t stlink_calculate_pagesize(stlink_t *sl, uint32_t flashaddr);
    uint16_t read_uint16(const unsigned char *c, const int pt);
    void stlink_core_stat(stlink_t *sl);
    void stlink_print_data(stlink_t *sl);
    unsigned int is_bigendian(void);
    uint32_t read_uint32(const unsigned char *c, const int pt);
    void write_uint32(unsigned char* buf, uint32_t ui);
    void write_uint16(unsigned char* buf, uint16_t ui);
    bool stlink_is_core_halted(stlink_t *sl);
    int write_buffer_to_sram(stlink_t *sl, flash_loader_t* fl, const uint8_t* buf, size_t size);
    int write_loader_to_sram(stlink_t *sl, stm32_addr_t* addr, size_t* size);
    int stlink_fread(stlink_t* sl, const char* path, bool is_ihex, stm32_addr_t addr, size_t size);
    int stlink_load_device_params(stlink_t *sl);

#include "stlink/sg.h"
#include "stlink/usb.h"
#include "stlink/reg.h"
#include "stlink/commands.h"
#include "stlink/chipid.h"
#include "stlink/flash_loader.h"

#include "stlink/version.h"

#ifdef __cplusplus
}
#endif

#endif /* STLINK_H */
