#include "fat.h"
#include "hardware/clocks.h"
#include "audio_i2s.pio.h"
#include "pico/audio_i2s.h"
#include "hardware/dma.h"
#include <hardware/pio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/sync.h>

struct fat_block_device fs = {
    .init = block_fs_init,
    .open = open_file
};

#define SAMPLES_PER_BUFFER 512
int16_t audio_buffer[2][SAMPLES_PER_BUFFER];
uint8_t play_buffer_index = 0;
static bool buffer_empty = false;
bool playing = false;

static void Audio_UpdateFrequency(uint32_t sample_freq)
{
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    uint32_t divider = system_clock_frequency * 4 / sample_freq;
    pio_sm_set_clkdiv_int_frac(pio0, 0, divider >> 8u, divider & 0xffu);
}

void __isr __time_critical_func(Audio_DMACallback)(void)
{
    if (dma_irqn_get_channel_status(0, 0))
    {
        dma_irqn_acknowledge_channel(0, 0);

        play_buffer_index = play_buffer_index ? 0 : 1;
        dma_channel_transfer_from_buffer_now(0, audio_buffer[play_buffer_index], SAMPLES_PER_BUFFER / 4);
        buffer_empty = true;
    }
}

struct audio_buffer_pool *init_audio() {

    static audio_format_t audio_format = {
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .sample_freq = 22050,
            .channel_count = 1,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 8
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const struct audio_format *output_format;
    struct audio_i2s_config config = {
            .data_pin = PICO_AUDIO_I2S_DATA_PIN,
            .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
            .dma_channel = 0,
            .pio_sm = 0,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);
    return producer_pool;
}

static int block_fs_init() {
    // reading master boot record
    fs.blk_dev->read_block(0);

    fs.partition_LBA = big_to_small_endian32(&(fs.blk_dev->in_buf[MBR_PARTITION_OFFSET + MBR_LBS_ABS_FIRST_SECTOR_OFFSET]));
    fs.num_sectors = big_to_small_endian32(&(fs.blk_dev->in_buf[MBR_PARTITION_OFFSET + MBR_NUM_SECTORS_IN_PARTITION_OFFSET]));

    fs.blk_dev->read_block(fs.partition_LBA);

    fs.num_reserved_sectors = big_to_small_endian16(&(fs.blk_dev->in_buf[FAT32_NUM_RESERVED_SECTORS_OFFSET]));
    fs.sectors_per_fat = big_to_small_endian32(&(fs.blk_dev->in_buf[FAT32_SECTORS_PER_FAT]));
    fs.sectors_per_cluster = fs.blk_dev->in_buf[FAT32_SECTORS_PER_CLUSTER_OFFSET];
    fs.root_dir_first_cluster = big_to_small_endian32(&(fs.blk_dev->in_buf[FAT32_ROOT_DIR_FIRST_CLUSTER]));

    fs.fat_begin_lba = fs.partition_LBA + fs.num_reserved_sectors;
    fs.cluster_begin_lba = fs.partition_LBA + fs.num_reserved_sectors + (FAT32_NUM_FATS * fs.sectors_per_fat);

    fs.fat_sector_offset = 0;

    // get files
    get_files();

    return 0;
}

static void get_files() {
    uint16_t first_cluster_high;
    uint16_t first_cluster_low;

    for (int sector_offset = 0; sector_offset < fs.sectors_per_cluster; sector_offset++) {
        for (int i = 0; i < 8; i++) { // 512 / 64 = 8
            struct file temp_file;

            fs.blk_dev->read_block(cluster_to_lba(fs.root_dir_first_cluster) + sector_offset);

            // for some reason my file is shifted by 0x40
            memcpy(temp_file.filename, &(fs.blk_dev->in_buf[0x20 + 0x40 * i + FAT32_DIR_NAME_OFFSET]), 12);

            // sanitise filename
            for (int i = 0; i < 11; i++) {
                // checking its not a space between words
                if (temp_file.filename[i] <= 0x20 && temp_file.filename[i+1] <= 0x20)
                    temp_file.filename[i] = '\0';
            }
            temp_file.filename[11] = '\0';

            temp_file.attribute = fs.blk_dev->in_buf[0x40 * i + FAT32_DIR_ATTR_OFFSET];
            // end of dir
            if (temp_file.attribute == 0)
                return;

            first_cluster_high = big_to_small_endian16(&(fs.blk_dev->in_buf[0x20 + 0x40 * i + FAT32_FIRST_CLUSTER_HIGH_OFFSET]));
            first_cluster_low = big_to_small_endian16(&(fs.blk_dev->in_buf[0x20 + 0x40 * i + FAT32_FIRST_CLUSTER_LOW_OFFSET]));
            temp_file.first_cluster = (first_cluster_high << 16) | first_cluster_low;
            temp_file.filesize = big_to_small_endian32(&(fs.blk_dev->in_buf[0x20 + 0x40 * i + FAT32_DIR_FILE_SIZE_OFFSET]));
            temp_file.file_fat_cluster_offset = 0;

            fs.files[i] = temp_file;
        }
    }
}

static int search_for_file(char filename[]) {
    // searching for file
    for (int i = 0; i < MAX_NUM_FILES; i++) {
        if(strcmp(filename, (const char *)fs.files[i].filename) == 0)
            return i;
    }
    return -1;
}

static int open_file(char filename[]) {
    int ret = 0;
    uint32_t bytes_read = 0;
    bool done_reading = false;

    // audio
    //struct audio_buffer_pool *ap = init_audio();
    //struct audio_buffer *buffer = take_audio_buffer(ap, true);
    //int16_t *samples = (int16_t *) buffer->buffer->bytes;
    // setup I2S pins
    gpio_set_function(PICO_AUDIO_I2S_DATA_PIN, GPIO_FUNC_PIO0);
    gpio_set_function(PICO_AUDIO_I2S_CLOCK_PIN_BASE, GPIO_FUNC_PIO0);

    // setup PIO0 state machine 0 with I2S program
    pio_sm_claim(pio0, 0);
    uint offset = pio_add_program(pio0, &audio_i2s_program);
    audio_i2s_program_init(pio0, 0, offset, PICO_AUDIO_I2S_DATA_PIN, PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    __mem_fence_release();

    // setup DMA channel
    dma_channel_claim(0);

    dma_channel_config dma_config = dma_channel_get_default_config(0);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_PIO0_TX0 + 0);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);

    dma_channel_configure(0,
                          &dma_config,
                          &pio0->txf[0],                 // dest:  TX buffer of PIO0, statemachine 0
                          NULL,                                 // src:   will be defined later
                          SAMPLES_PER_BUFFER / 4,                      // count: half of the buffer (counted in 32bit words)
                          false);                               // don't trigger immediately

    irq_add_shared_handler(DMA_IRQ_0, Audio_DMACallback, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    dma_irqn_set_channel_enabled(0, 0, true);
    irq_set_enabled(DMA_IRQ_0, true);

    // Set correct I2S frequency
    Audio_UpdateFrequency(22050);
    __mem_fence_release();

    // Connect buffer
    play_buffer_index = 0;
    buffer_empty = true;
    playing = true;

    ret = search_for_file(filename);
    if (ret < 0) {
        Log(LOG_ERROR, "Could not find file", -1);
        return -1;
    }
    Log(LOG_DEBUG, "Found file", 0);

    printf("Filename: %s\n", fs.files[ret].filename);
    printf("Attribute: %d\n", fs.files[ret].attribute);
    printf("First cluster: %x\n", fs.files[ret].first_cluster);
    printf("File size: %d\n", fs.files[ret].filesize);

    // reading file header
    uint32_t cluster = fs.files[ret].first_cluster;
    fs.blk_dev->read_block(cluster_to_lba(cluster));

    printf("File size: %d\n", big_to_small_endian32(&(fs.blk_dev->in_buf[4])));
    printf("Length of format data: %d\n", big_to_small_endian32(&(fs.blk_dev->in_buf[16])));
    printf("Type of format: %d\n", big_to_small_endian16(&(fs.blk_dev->in_buf[20])));
    printf("Channels: %d\n", big_to_small_endian16(&(fs.blk_dev->in_buf[22])));
    printf("Sample rate: %d\n", big_to_small_endian32(&(fs.blk_dev->in_buf[24])));
    printf("Sample rate * BitsPerSample * Channels / 8: %d\n", big_to_small_endian32(&(fs.blk_dev->in_buf[28])));
    printf("BitsPerSample * Channels / 8: %d\n", big_to_small_endian16(&(fs.blk_dev->in_buf[32])));
    printf("BitsPerSample: %d\n", big_to_small_endian16(&(fs.blk_dev->in_buf[34])));
    printf("Data section size: %d\n", big_to_small_endian32(&(fs.blk_dev->in_buf[40])));
    pio_sm_set_enabled(pio0, 0, true);

    // start reading file
    //uint32_t cluster = fs.files[ret].first_cluster;
    fs.fat_sector_offset = cluster / FAT32_ENTRIES_PER_FAT;
    fs.blk_dev->read_block(fs.fat_begin_lba + fs.fat_sector_offset);
    do {
        for (int sector_offset = 0; sector_offset < fs.sectors_per_cluster; sector_offset++) {
            fs.blk_dev->read_block(cluster_to_lba(cluster) + sector_offset);
            memcpy((uint8_t*) audio_buffer, fs.blk_dev->in_buf, SAMPLES_PER_BUFFER);
            bytes_read += 512;

            if (bytes_read >= fs.files[ret].filesize) {
                done_reading = true;
                break;
            }

            dma_channel_transfer_from_buffer_now(0, audio_buffer[play_buffer_index], SAMPLES_PER_BUFFER / 4);
            pio_sm_set_enabled(pio0, 0, true);


            //for (uint i = 0; i < buffer->max_sample_count; i+=2) {
            //    samples[i] = (fs.blk_dev->in_buf[i]) | (fs.blk_dev->in_buf[i+1] << 8);
            //}

            //buffer->sample_count = buffer->max_sample_count;
            //give_audio_buffer(ap, buffer);

        }

        // check fat
        fs.blk_dev->read_block(fs.fat_begin_lba + fs.fat_sector_offset);
        cluster = big_to_small_endian32(&(fs.blk_dev->in_buf[(cluster * 4) % FAT32_ENTRIES_PER_FAT]));
        fs.fat_sector_offset = cluster / FAT32_ENTRIES_PER_FAT;
    } while(cluster < 0x0ffffff8 || !done_reading);

    Log(LOG_DEBUG, "Done reading file", 0);

    return 0;
};

uint32_t cluster_to_lba(uint32_t cluster) {
    return fs.cluster_begin_lba + (cluster - 2) * fs.sectors_per_cluster;
}

uint32_t big_to_small_endian32(uint8_t num[]) {
    return num[3] << 24 | num[2] << 16 | num[1] << 8 | num[0];
}

uint16_t big_to_small_endian16(uint8_t num[]) {
    return num[1] << 8 | num[0];
}

