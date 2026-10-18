// The board's real display.c on the host: the panel is a frame buffer we can save.
// (Linker sections mean nothing here, and Mach-O rejects ELF section names.)
#define section(name) unused
#include "../../board/DevKit-E8/display.c"
#undef section

static int32_t ok_init(void (*cb)(uint32_t)) { (void)cb; return 0; }
static int32_t ok_power(int s) { (void)s; return 0; }
static int32_t ok_control(uint32_t c, uint32_t a) { (void)c; (void)a; return 0; }
static int32_t ok_start(void) { return 0; }
ARM_DRIVER_CDC200 Driver_CDC200 = {ok_init, ok_power, ok_control, ok_start};

// Saves the panel as a binary PPM (the frame buffer holds B, G, R).
void display_host_save(const char *path)
{
    FILE *f = fopen(path, "wb");
    fprintf(f, "P6\n%d %d\n255\n", LCD_WIDTH, LCD_HEIGHT);
    for (int y = 0; y < LCD_HEIGHT; y++)
        for (int x = 0; x < LCD_WIDTH; x++) {
            uint8_t px[3] = {lcd_frame[y][x][2], lcd_frame[y][x][1], lcd_frame[y][x][0]};
            fwrite(px, 1, 3, f);
        }
    fclose(f);
}
