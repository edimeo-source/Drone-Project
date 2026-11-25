/* =============================================================================================
 *  ELC275 DRONE PROJECT  -  Ethan DiMeo
 *  File    : firmware/01_bench_tests/09_bluefruit_relay_test/bluefruit_relay_test.c
 *  What    : Bluefruit + relay test - phone START/STOP switches the motor-power relay
 *  Version : bench test   |   Date: 2026-11-23
 *  Status  : WORKED: the relay clicks on START and off on STOP from the phone.
 *  Board   : PIC32MZ2048EFM144 EF Starter Kit, MPLAB X 6.15, XC32, config.bits (120 MHz SYSCLK,
 *            60 MHz PBCLK)
 * ---------------------------------------------------------------------------------------------
 *  J12 PINOUT FOR THIS VERSION  (PIC32MZ EF Starter Kit, J12 header)
 *    Bluefruit SPI Friend
 *      VIN pin 2 (+5 V)   GND pin 6   SCK pin 23 (RD1)   MOSI pin 37 (RD10)   MISO pin 7
 *      (RA14)
 *      CS pin 13 (RJ13)   IRQ and RST not connected
 *    IMU CS pin 16 (RK1) is held HIGH so the IMU stays off the bus
 *    Motor-power relay . CH1 = J12 pin 33 (RK4), HIGH = motor power ON, LOW/reset = power cut
 *    Power/relay board <-> kit: pin 2 (+5 V from the 7805 INTO the kit), pin 33 (RK4), pin 34
 *      (GND)
 *    Kit LEDs LED1-3 = RH0-RH2   SW1 = RB12   SW2 = RB13
 *    PuTTY . UART2 over the J11 USB port, 9600 baud 8N1 (this test runs on the 8 MHz FRC
 *      clock)
 * ---------------------------------------------------------------------------------------------
 *  FUNCTIONS IN THIS FILE (each has a title block above it): now_ticks(), delay_us(),
 *    delay_ms(), reset_word(), set_led(), relay_off(), relay_service(), early_gpio(),
 *    wait_clock(), setup_uart(), log_text(), log_hex(), log_spi_failure(), spi_byte(),
 *    select_bluefruit(), release_bluefruit(), sdep_send(), sdep_read(), at_command(),
 *    drain_old_sdep(), finish_word(), receive_phone_text(), phone_reply(), apply_command(),
 *    main()
 * ---------------------------------------------------------------------------------------------
 *  HOW TO USE: new MPLAB X project (PIC32MZ2048EFM144, XC32). Copy this whole file into main.c.
 *  This file carries its own #pragma config lines (FRC 8 MHz): do NOT add config.bits.
 * ============================================================================================ */

#include <xc.h>
#include <stdint.h>
#include <string.h>

#pragma config FNOSC = FRCDIV
#pragma config POSCMOD = OFF
#pragma config FSOSCEN = OFF
#pragma config IESO = OFF
#pragma config FWDTEN = OFF
#pragma config FDMTEN = OFF
#pragma config IOL1WAY = OFF
#pragma config FCKSM = CSECMD
#pragma config JTAGEN = OFF
#pragma config TRCEN = OFF

/* RELAY V4: START turns the relay on; STOP turns it off.
 * Phone <-> Bluefruit <-> PIC32 <-> relay driver. No timer interrupts.
 * PIC32MZ2048EFM144 EF Starter Kit. FRC 8 MHz. PuTTY 9600 8N1.
 * J12: Bluefruit VIN=2, GND=6, SCK=23/RD1, MOSI=37/RD10,
 * MISO=7/RA14, CS=13/RJ13. IRQ/RST unconnected. IMU CS=16 HIGH.
 * Relay driver 33/RK4: START holds ON until STOP or a detected fault.
 * HIGH-active transistor driver assumed. Motor power contacts disconnected.
 * STATUS reports commanded relay state. No motor PWM.
 */
#define RELAY_ACTIVE_HIGH 1
#define RELAY_PIN (1u << 4)
#define SCK_PIN (1u << 1)
#define MOSI_PIN (1u << 10)
#define MISO_PIN (1u << 14)
#define CS_PIN (1u << 13)
#define RED_LED 1u
#define YELLOW_LED 2u
#define GREEN_LED 4u
#define MS_TICKS(ms) ((uint32_t)(ms) * 4000u)
#define AT_TIMEOUT_MS 500u

static unsigned green_on;
static unsigned relay_active, connected, drain_phone = 1, relay_event;
static uint32_t link_checked;
static void relay_service(void);
static char response[256];
static char word[16];
static unsigned word_length, discard_word;
static uint32_t word_updated;
static unsigned diag_tx = 0x100u, diag_rx = 0x100u, diag_error;
static char diag_command[80];

// ---------------------------------------------------------------------------------------------
// static uint32_t now_ticks(void)
//   Returns the core-timer count (4 MHz with the 8 MHz FRC clock in this test).
// ---------------------------------------------------------------------------------------------
static uint32_t now_ticks(void)
{
    return _CP0_GET_COUNT();
}

// ---------------------------------------------------------------------------------------------
// static void delay_us(uint32_t us)
//   Waits us microseconds on the core timer, servicing the relay safety check while it waits.
// ---------------------------------------------------------------------------------------------
static void delay_us(uint32_t us)
{
    relay_service();
    uint32_t start = now_ticks();
    while ((uint32_t)(now_ticks() - start) < us * 4u) relay_service();
}

// ---------------------------------------------------------------------------------------------
// static void delay_ms(unsigned ms)
//   Waits n milliseconds (busy loop on a timer; 234 Timer3 counts = 1 ms).
// ---------------------------------------------------------------------------------------------
static void delay_ms(unsigned ms)
{
    while (ms--) delay_us(1000);
}

// ---------------------------------------------------------------------------------------------
// static void reset_word(void)
//   Clears the word being collected from the phone.
// ---------------------------------------------------------------------------------------------
static void reset_word(void)
{
    word_length = discard_word = 0;
    word[0] = '\0';
}

// ---------------------------------------------------------------------------------------------
// static void set_led(unsigned green)
//   Green LED = relay on, red LED = relay off.
// ---------------------------------------------------------------------------------------------
static void set_led(unsigned green)
{
    green_on = green;
    LATHCLR = RED_LED | GREEN_LED;
    LATHSET = green ? GREEN_LED : RED_LED;
}

// ---------------------------------------------------------------------------------------------
// static void relay_off(void)
//   Opens the relay (motor power off) and remembers it is off.
// ---------------------------------------------------------------------------------------------
static void relay_off(void)
{
    relay_active = 0;
#if RELAY_ACTIVE_HIGH
    LATKCLR = RELAY_PIN;
#else
    LATKSET = RELAY_PIN;
#endif
    set_led(0);
}

// ---------------------------------------------------------------------------------------------
// static void relay_service(void)
//   Safety check: opens the relay if the phone link is lost, not checked for 1 s, or SW1 is
//   pressed.
// ---------------------------------------------------------------------------------------------
static void relay_service(void)
{
    if (!relay_active) return;
    uint32_t now = now_ticks();
    if (!connected || (uint32_t)(now - link_checked) >= MS_TICKS(1000) ||
        !(PORTB & (1u << 12))) {
        relay_off();
        drain_phone = 1;
        relay_event = 1;
    }
}

// ---------------------------------------------------------------------------------------------
// static void early_gpio(void)
//   First thing at reset: interrupts off, LEDs and relay pin set to a safe state (relay OFF).
// ---------------------------------------------------------------------------------------------
static void early_gpio(void)
{
    __builtin_disable_interrupts();
    ANSELHCLR = 3u;
    LATHCLR = 7u;
    LATHSET = RED_LED;
    TRISHCLR = 7u;
    relay_off();                 /* Set OFF latch before enabling output. */
    TRISKCLR = RELAY_PIN;
    LATKSET = (1u << 1);          /* IMU deselected. */
    TRISKCLR = (1u << 1);
    LATJSET = CS_PIN;
    TRISJCLR = CS_PIN;
    SPI1CON = 0;
    OC1CON = OC2CON = OC3CON = OC4CON = 0;
    T2CON = T4CON = 0;
    ANSELDCLR = SCK_PIN | MOSI_PIN | 1u;
    ANSELFCLR = 0x34u;
    LATDCLR = SCK_PIN | MOSI_PIN | 1u;
    LATFCLR = 0x34u;
    TRISDCLR = SCK_PIN | MOSI_PIN | 1u;
    TRISFCLR = 0x34u;             /* All ESC signals held LOW. */
    TRISASET = MISO_PIN;
    ANSELBCLR = (1u << 12) | (1u << 14);
    TRISBSET = (1u << 12);
    CNPUBSET = (1u << 12);
}

// ---------------------------------------------------------------------------------------------
// static int wait_clock(volatile uint32_t *divider)
//   Waits (up to 100 ms) for a peripheral-bus clock divider to finish changing. 1 = ready, 0 =
//   timed out.
// ---------------------------------------------------------------------------------------------
static int wait_clock(volatile uint32_t *divider)
{
    uint32_t start = now_ticks();
    while (!(*divider & 0x800u)) {
        if ((uint32_t)(now_ticks() - start) >= MS_TICKS(100)) return 0;
    }
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static int setup_uart(void)
//   Unlocks the system, sets the clock dividers and starts UART2 at 9600 baud for PuTTY. 1 =
//   OK.
// ---------------------------------------------------------------------------------------------
static int setup_uart(void)
{
    SYSKEY = 0;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    OSCCONbits.FRCDIV = 0;
    if (!wait_clock(&PB2DIV)) { SYSKEY = 0; return 0; }
    PB2DIVbits.PBDIV = 1;
    if (!wait_clock(&PB2DIV)) { SYSKEY = 0; return 0; }
    PB2DIVbits.ON = 1;
    CFGCONbits.IOLOCK = 0;
    RPB14R = 2;
    RPD10R = RPD0R = RPF2R = RPF4R = RPF5R = 0;
    CFGCONbits.IOLOCK = 1;
    SYSKEY = 0;
    LATBSET = (1u << 14);
    TRISBCLR = (1u << 14);
    U2MODE = U2STA = 0;
    U2BRG = 25;
    U2MODEbits.ON = 1;
    U2STAbits.UTXEN = 1;
    return 1;
}

// ---------------------------------------------------------------------------------------------
// static void log_text(const char *text)
//   Prints text to PuTTY (servicing the relay check while it waits for the UART).
// ---------------------------------------------------------------------------------------------
static void log_text(const char *text)
{
    while (*text) {
        relay_service();
        uint32_t start = now_ticks();
        while (U2STAbits.UTXBF || !U2STAbits.TRMT) {
            relay_service();
            if ((uint32_t)(now_ticks() - start) >= MS_TICKS(10)) return;
        }
        U2TXREG = (uint8_t)*text++;
    }
}

// ---------------------------------------------------------------------------------------------
// static void log_hex(unsigned value)
//   Prints a byte as two hex digits (-- if it is not a byte).
// ---------------------------------------------------------------------------------------------
static void log_hex(unsigned value)
{
    const char digits[] = "0123456789ABCDEF";
    char hex[3];
    if (value > 255u) { log_text("--"); return; }
    hex[0] = digits[(value >> 4) & 15u];
    hex[1] = digits[value & 15u];
    hex[2] = '\0';
    log_text(hex);
}

// ---------------------------------------------------------------------------------------------
// static void log_spi_failure(void)
//   Prints what went wrong in the last SPI exchange with the Bluefruit (command, first bytes,
//   reason).
// ---------------------------------------------------------------------------------------------
static void log_spi_failure(void)
{
    log_text("SPI FAIL CMD=");
    log_text(diag_command);
    log_text(" TX_FIRST=0x"); log_hex(diag_tx);
    log_text(" RX_FIRST=0x"); log_hex(diag_rx);
    log_text(" REASON=");
    switch (diag_error) {
        case 1: log_text("TX BUSY TIMEOUT"); break;
        case 2: log_text("RX TIMEOUT"); break;
        case 3: log_text("BAD SDEP HEADER"); break;
        case 4: log_text("SDEP ERROR PACKET"); break;
        case 5: log_text("WRONG SDEP COMMAND ID"); break;
        case 6: log_text("REPLY TOO LONG"); break;
        case 7: log_text("AT REPLY NOT OK"); break;
        default: log_text("UNKNOWN"); break;
    }
    log_text("\r\n");
}

// ---------------------------------------------------------------------------------------------
// static uint8_t spi_byte(uint8_t value)
//   One SPI byte out and in by bit-banging the pins (mode 0), so this test needs no SPI module
//   setup.
// ---------------------------------------------------------------------------------------------
static uint8_t spi_byte(uint8_t value)
{
    uint8_t received = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        if (value & 0x80u) LATDSET = MOSI_PIN;
        else LATDCLR = MOSI_PIN;
        delay_us(4);
        LATDSET = SCK_PIN;
        received = (uint8_t)((received << 1) | ((PORTA & MISO_PIN) != 0));
        delay_us(4);
        LATDCLR = SCK_PIN;
        value <<= 1;
    }
    return received;
}

// ---------------------------------------------------------------------------------------------
// static void select_bluefruit(void)
//   Pulls the Bluefruit's CS low and waits the 110 us the module needs.
// ---------------------------------------------------------------------------------------------
static void select_bluefruit(void)
{
    LATJCLR = CS_PIN;
    delay_us(110);
}

// ---------------------------------------------------------------------------------------------
// static void release_bluefruit(void)
//   Raises the Bluefruit's CS.
// ---------------------------------------------------------------------------------------------
static void release_bluefruit(void)
{
    LATJSET = CS_PIN;
    delay_us(20);
}

// ---------------------------------------------------------------------------------------------
// static int sdep_send(const uint8_t *payload, unsigned length, unsigned more)
//   Sends one SDEP packet (up to 16 payload bytes) to the Bluefruit, waiting while it says
//   busy.
// ---------------------------------------------------------------------------------------------
static int sdep_send(const uint8_t *payload, unsigned length, unsigned more)
{
    uint32_t start = now_ticks();
    do {
        select_bluefruit();
        uint8_t ready = spi_byte(0x10);
        diag_tx = ready;
        /* Adafruit's transmitter retries 0xFE only. 0xFF is allowed here. */
        if (ready != 0xFE) {
            spi_byte(0x00);
            spi_byte(0x0A);
            spi_byte((uint8_t)(length | (more ? 0x80u : 0u)));
            for (unsigned i = 0; i < length; ++i) spi_byte(payload[i]);
            release_bluefruit();
            return 1;
        }
        release_bluefruit();
        delay_ms(1);
    } while ((uint32_t)(now_ticks() - start) < MS_TICKS(AT_TIMEOUT_MS));
    diag_error = 1;
    return 0;
}

// ---------------------------------------------------------------------------------------------
// static int sdep_read(unsigned *type, unsigned *id, unsigned *size, uint8_t *data)
//   Reads one SDEP packet from the Bluefruit (type, id, size, data). 1 = got one.
// ---------------------------------------------------------------------------------------------
static int sdep_read(unsigned *type, unsigned *id, unsigned *size, uint8_t *data)
{
    select_bluefruit();
    *type = spi_byte(0xFF);
    diag_rx = *type;
    if (*type == 0xFE || *type == 0xFF) {
        release_bluefruit();
        return 0;
    }
    unsigned low = spi_byte(0xFF);
    unsigned high = spi_byte(0xFF);
    *id = low | (high << 8);
    *size = spi_byte(0xFF);
    unsigned length = *size & 0x7Fu;
    if (length > 16 || (*size & 0x60u) ||
        (*type != 0x20 && *type != 0x40 && *type != 0x80)) {
        release_bluefruit();
        diag_error = 3;
        return -1;
    }
    for (unsigned i = 0; i < length; ++i) data[i] = spi_byte(0xFF);
    release_bluefruit();
    if (*type == 0x80) diag_error = 4;
    return *type == 0x80 ? -1 : 1;
}

// ---------------------------------------------------------------------------------------------
// static int at_command(const char *command)
//   Sends one AT command and collects the reply text into response[]. 1 = OK.
// ---------------------------------------------------------------------------------------------
static int at_command(const char *command)
{
    diag_tx = diag_rx = 0x100u;
    diag_error = 0;
    strncpy(diag_command, command, sizeof diag_command - 1);
    diag_command[sizeof diag_command - 1] = '\0';
    response[0] = '\0';
    unsigned remaining = (unsigned)strlen(command);
    const uint8_t *data = (const uint8_t *)command;
    while (remaining) {
        unsigned count = remaining > 16 ? 16 : remaining;
        if (!sdep_send(data, count, remaining > count)) return 0;
        data += count;
        remaining -= count;
    }
    unsigned used = 0;
    uint32_t start = now_ticks();
    response[0] = '\0';
    while ((uint32_t)(now_ticks() - start) < MS_TICKS(AT_TIMEOUT_MS)) {
        unsigned type, id, size;
        uint8_t packet[16];
        int result = sdep_read(&type, &id, &size, packet);
        if (result < 0) return 0;
        if (!result) { delay_ms(1); continue; }
        if (type != 0x20) continue; /* Ignore alerts. */
        if (id != 0x0A00) { diag_error = 5; return 0; }
        unsigned count = size & 0x7Fu;
        if (used + count >= sizeof response) { diag_error = 6; return 0; }
        memcpy(response + used, packet, count);
        used += count;
        if (size & 0x80u) continue;
        response[used] = '\0';
        while (used && (response[used-1] == '\r' || response[used-1] == '\n'))
            response[--used] = '\0';
        if (used < 2 || strcmp(response + used - 2, "OK") != 0) {
            diag_error = 7; return 0;
        }
        if (used > 2 && response[used-3] != '\r' && response[used-3] != '\n') {
            diag_error = 7; return 0;
        }
        response[used -= 2] = '\0';
        while (used && (response[used-1] == '\r' || response[used-1] == '\n'))
            response[--used] = '\0';
        unsigned leading = 0;
        while (response[leading] == '\r' || response[leading] == '\n') ++leading;
        if (leading) memmove(response, response + leading, used - leading + 1);
        return 1;
    }
    diag_error = 2;
    return 0;
}

// ---------------------------------------------------------------------------------------------
// static void drain_old_sdep(void)
//   Throws away any old SDEP replies still waiting in the module (500 ms).
// ---------------------------------------------------------------------------------------------
static void drain_old_sdep(void)
{
    uint32_t start = now_ticks();
    while ((uint32_t)(now_ticks() - start) < MS_TICKS(500)) {
        unsigned type, id, size;
        uint8_t data[16];
        sdep_read(&type, &id, &size, data);
        delay_ms(2);
    }
}

// ---------------------------------------------------------------------------------------------
// static unsigned finish_word(void)
//   Ends the current word and turns it into a request (START / STOP / STATUS).
// ---------------------------------------------------------------------------------------------
static unsigned finish_word(void)
{
    unsigned request = 0;
    if (!discard_word) {
        if (!strcmp(word, "START")) request = 1;
        else if (!strcmp(word, "STOP")) request = 2;
        else if (!strcmp(word, "STATUS")) request = 3;
        else if (word_length) request = 4;
    } else request = 4;
    reset_word();
    return request;
}

// ---------------------------------------------------------------------------------------------
// static unsigned receive_phone_text(const char *text)
//   Splits phone text into words and returns the request: 1 = START, 2 = STOP, 3 = STATUS, 0 =
//   none.
// ---------------------------------------------------------------------------------------------
static unsigned receive_phone_text(const char *text)
{
    unsigned request = 0;
    for (; *text; ++text) {
        char c = *text;
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        if (c == ';' || c == '\r' || c == '\n' || c == ' ' || c == '\t') {
            unsigned next = finish_word();
            if (next == 2) return 2; /* STOP beats START in this batch. */
            if (next == 4 || request == 4) request = 4;
            else if (next == 1 || !request) request = next;
            continue;
        }
        word_updated = now_ticks();
        if (discard_word) continue;
        if (word_length >= sizeof word - 1) {
            discard_word = 1;
            continue;
        }
        word[word_length++] = c;
        word[word_length] = '\0';
        if (!strcmp(word, "STOP")) { reset_word(); return 2; }
    }
    /* Accept a complete bare command in this reply without a 200 ms wait. */
    if (!discard_word && (!strcmp(word, "START") || !strcmp(word, "STATUS"))) {
        unsigned next = finish_word();
        if (request != 4 && (next == 1 || !request)) request = next;
    }
    return request;
}

// ---------------------------------------------------------------------------------------------
// static int phone_reply(const char *text)
//   Sends a text line to the phone with AT+BLEUARTTX.
// ---------------------------------------------------------------------------------------------
static int phone_reply(const char *text)
{
    char command[80];
    if (strlen(text) > 48) return 0;
    strcpy(command, "AT+BLEUARTTX=");
    strcat(command, text);
    log_text("PIC TX: ");
    log_text(text);
    log_text("\r\n");
    return at_command(command);
}

// ---------------------------------------------------------------------------------------------
// static int apply_command(unsigned request)
//   Carries out a phone request: START closes the relay (only if every check passes), STOP
//   opens it.
// ---------------------------------------------------------------------------------------------
static int apply_command(unsigned request)
{
    relay_service();
    if (request == 1) {
        if (drain_phone || relay_event || !connected ||
            (uint32_t)(now_ticks() - link_checked) >= MS_TICKS(1000) ||
            !(PORTB & (1u << 12))) {
            relay_off();
            return phone_reply("PIC RELAY OFF");
        }
        if (!relay_active) {
            relay_active = 1;
#if RELAY_ACTIVE_HIGH
            LATKSET = RELAY_PIN;
#else
            LATKCLR = RELAY_PIN;
#endif
            set_led(1);
        }
        return phone_reply(relay_active ? "PIC RELAY ON" : "PIC RELAY OFF");
    }
    if (request == 2) {
        relay_off();
        drain_phone = 1;
        reset_word();
        return phone_reply("PIC RELAY OFF");
    }
    if (request == 3)
        return phone_reply(relay_active ? "PIC RELAY ON" : "PIC RELAY OFF");
    if (request == 4) {
        relay_off();
        drain_phone = 1;
        return phone_reply("PIC USE START STOP STATUS");
    }
    return 1;
}

// ---------------------------------------------------------------------------------------------
// int main(void)
//   Program entry: safe GPIO, clock and UART setup, Bluefruit start-up, then phone
//   START/STOP/STATUS controls the relay.
// ---------------------------------------------------------------------------------------------
int main(void)
{
    early_gpio();
    for (unsigned i = 0; i < 3; ++i) {
        LATHSET = RED_LED; delay_ms(150);
        LATHCLR = RED_LED; delay_ms(150);
    }
    relay_off();
    if (!setup_uart()) {
        for (;;) {
            LATHINV = RED_LED | YELLOW_LED;
            delay_ms(250);
        }
    }
    log_text("RELAY V4 BOOT OK. START / STOP. 9600 baud.\r\n");
    for (;;) {
        relay_off();
        connected = relay_event = 0;
        drain_phone = 1;
        reset_word();
        LATHSET = YELLOW_LED;
        delay_ms(1000);
        drain_old_sdep();
        if (!at_command("ATE=0") || !at_command("AT")) {
            log_spi_failure();
            continue;
        }
        LATHCLR = YELLOW_LED;
        log_text("SPI PASS. Connect phone UART; START, STOP, STATUS.\r\n");
        uint32_t last_check = now_ticks() - MS_TICKS(250);
        uint32_t last_notice = now_ticks();
        for (;;) {
            relay_service();
            if (relay_event) {
                log_text("Relay automatically OFF.\r\n");
                reset_word();
                relay_event = 0;
            }
            if ((uint32_t)(now_ticks() - last_check) >= MS_TICKS(250)) {
                last_check = now_ticks();
                if (!at_command("AT+GAPGETCONN")) break;
                unsigned current = !strcmp(response, "1");
                if (current) link_checked = now_ticks();
                if (current != connected) {
                    connected = current;
                    relay_off();
                    drain_phone = 1;
                    reset_word();
                    log_text(connected ? "PHONE CONNECTED.\r\n" : "PHONE DISCONNECTED. Relay OFF.\r\n");
                }
            }
            if ((uint32_t)(now_ticks() - last_notice) >= MS_TICKS(2000)) {
                last_notice = now_ticks();
                log_text(connected ? "PIC RUNNING: phone connected.\r\n" : "PIC RUNNING: waiting for phone.\r\n");
            }
            if (!connected) { delay_ms(20); continue; }
            if (!at_command("AT+BLEUARTRX")) break;
            relay_service();
            if (drain_phone || relay_event) {
                reset_word();
                if (!response[0]) {
                    drain_phone = relay_event = 0;
                    if (!phone_reply("PIC RELAY OFF - READY")) break;
                }
                delay_ms(20);
                continue;
            }
            if (response[0]) {
                log_text("PHONE RX: ");
                log_text(response);
                log_text("\r\n");
            }
            unsigned request = receive_phone_text(response);
            if (!request && word_length &&
                (uint32_t)(now_ticks() - word_updated) >= MS_TICKS(200))
                request = finish_word();
            if (request && !apply_command(request)) break;
            delay_ms(20);
        }
        relay_off();
        connected = 0;
        drain_phone = 1;
        LATHSET = YELLOW_LED;
        log_spi_failure();
        log_text("Relay OFF. Restarting communication.\r\n");
    }
}
