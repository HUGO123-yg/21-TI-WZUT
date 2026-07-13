#include "Flash.h"

#include <float.h>
#include <string.h>

#include "config.h"
#include "zf_driver_flash.h"

// Navigation data pages keep 500 samples at the front and a self-describing
// footer at the end. The commit marker is word 511, so an interrupted sequential
// write cannot accidentally look like a complete page.
#define NAV_FLASH_DATA_MAGIC_WORD             (NAV_FLASH_SAMPLES_PER_PAGE + 0U)
#define NAV_FLASH_DATA_VERSION_WORD           (NAV_FLASH_SAMPLES_PER_PAGE + 1U)
#define NAV_FLASH_DATA_ROUTE_WORD             (NAV_FLASH_SAMPLES_PER_PAGE + 2U)
#define NAV_FLASH_DATA_SEQUENCE_WORD          (NAV_FLASH_SAMPLES_PER_PAGE + 3U)
#define NAV_FLASH_DATA_COUNT_WORD             (NAV_FLASH_SAMPLES_PER_PAGE + 4U)
#define NAV_FLASH_DATA_GENERATION_WORD        (NAV_FLASH_SAMPLES_PER_PAGE + 5U)
#define NAV_FLASH_DATA_CRC_WORD               (NAV_FLASH_SAMPLES_PER_PAGE + 6U)
#define NAV_FLASH_DATA_CRC_INPUT_WORDS        (NAV_FLASH_DATA_CRC_WORD)
#define NAV_FLASH_COMMIT_WORD                 (FLASH_PAGE_LENGTH - 1U)

// Metadata uses the first words for a route directory and the final two words
// for CRC and commit. All unused words remain 0xFFFFFFFF and are covered by CRC.
#define NAV_FLASH_META_MAGIC_WORD             (0U)
#define NAV_FLASH_META_VERSION_WORD           (1U)
#define NAV_FLASH_META_GENERATION_WORD        (2U)
#define NAV_FLASH_META_ROUTE_COUNT_WORD       (3U)
#define NAV_FLASH_META_ROUTE_BASE_WORD        (4U)
#define NAV_FLASH_META_ROUTE_WORDS            (5U)
#define NAV_FLASH_META_ROUTE_VALID_OFFSET     (0U)
#define NAV_FLASH_META_ROUTE_START_OFFSET     (1U)
#define NAV_FLASH_META_ROUTE_END_OFFSET       (2U)
#define NAV_FLASH_META_ROUTE_COUNT_OFFSET     (3U)
#define NAV_FLASH_META_ROUTE_GENERATION_OFFSET (4U)
#define NAV_FLASH_META_CRC_WORD               (FLASH_PAGE_LENGTH - 2U)
#define NAV_FLASH_META_CRC_INPUT_WORDS        (NAV_FLASH_META_CRC_WORD)

#define NAV_FLASH_NO_PAGE                     (0xFFU)
#define NAV_FLASH_NO_BUFFER                   (0xFFU)

typedef struct
{
    uint8 start_page;
    uint8 end_page;
} nav_flash_route_layout_t;

typedef struct
{
    uint32 valid;
    uint32 start_page;
    uint32 end_page;
    uint32 sample_count;
    uint32 data_generation;
} nav_flash_route_metadata_t;

typedef struct
{
    uint32 generation;
    nav_flash_route_metadata_t route[NAV_FLASH_ROUTE_COUNT];
} nav_flash_metadata_t;

static const nav_flash_route_layout_t route_layout[NAV_FLASH_ROUTE_COUNT] =
{
    {NAV_FLASH_ROUTE1_START_PAGE, NAV_FLASH_ROUTE1_END_PAGE},
    {NAV_FLASH_ROUTE2_START_PAGE, NAV_FLASH_ROUTE2_END_PAGE},
    {NAV_FLASH_ROUTE3_START_PAGE, NAV_FLASH_ROUTE3_END_PAGE}
};

// Two recording buffers allow an ISR to keep collecting samples while the main
// loop writes the other buffer. Only pending_buffer is accessed by the writer.
static uint32 record_buffer[2][FLASH_PAGE_LENGTH];
static uint32 verify_buffer[FLASH_PAGE_LENGTH];
static uint32 io_buffer[FLASH_PAGE_LENGTH];
static int32 replay_samples[NAV_FLASH_REPLAY_MAX_SAMPLES];

static nav_flash_metadata_t metadata;
static volatile nav_flash_state_t module_state = NAV_FLASH_STATE_UNINITIALIZED;
static volatile nav_flash_status_t last_status = NAV_FLASH_STATUS_NOT_INITIALIZED;
static uint8 active_metadata_page = NAV_FLASH_NO_PAGE;
static uint8 active_record_buffer;
static volatile uint8 pending_buffer = NAV_FLASH_NO_BUFFER;
static uint8 pending_page;
static uint8 record_route_id;
static uint8 finish_requested;
static uint16 active_sample_count;
static uint16 next_page_sequence;
static uint32 record_sample_count;
static uint32 record_generation;
static float record_distance;
static uint8 replay_route_id;
static uint32 replay_sample_count;

static uint32 nav_flash_route_page_count(uint8 route_index)
{
    return (uint32)route_layout[route_index].start_page
           - (uint32)route_layout[route_index].end_page + 1U;
}

static uint32 nav_flash_route_capacity(uint8 route_index)
{
    return nav_flash_route_page_count(route_index)
           * NAV_FLASH_SAMPLES_PER_PAGE;
}

static uint8 nav_flash_route_id_is_valid(uint8 route_id)
{
    return (uint8)((route_id >= 1U) && (route_id <= NAV_FLASH_ROUTE_COUNT));
}

static uint8 nav_flash_page_is_in_route(uint8 page, uint8 route_index)
{
    return (uint8)((page <= route_layout[route_index].start_page)
                   && (page >= route_layout[route_index].end_page));
}

static uint8 nav_flash_config_is_valid(void)
{
    uint8 route_index;
    uint8 other_index;

    if ((NAV_FLASH_ROUTE_COUNT != 3U)
        || (NAV_FLASH_META_PAGE_A == NAV_FLASH_META_PAGE_B)
        || (NAV_FLASH_META_PAGE_A >= FLASH_PAGE_NUM)
        || (NAV_FLASH_META_PAGE_B >= FLASH_PAGE_NUM)
        || (NAV_FLASH_SAMPLES_PER_PAGE == 0U)
        || (NAV_FLASH_DATA_CRC_WORD >= NAV_FLASH_COMMIT_WORD)
        || (NAV_FLASH_META_ROUTE_BASE_WORD
            + NAV_FLASH_ROUTE_COUNT * NAV_FLASH_META_ROUTE_WORDS
            >= NAV_FLASH_META_CRC_WORD)
        || (NAV_FLASH_SAMPLE_DISTANCE <= 0.0f)
        || (NAV_FLASH_YAW_SCALE <= 0.0f)
        || (NAV_FLASH_WRITE_RETRY_COUNT == 0U))
    {
        return 0;
    }

    for (route_index = 0; route_index < NAV_FLASH_ROUTE_COUNT; route_index++)
    {
        if ((route_layout[route_index].start_page >= FLASH_PAGE_NUM)
            || (route_layout[route_index].end_page >= FLASH_PAGE_NUM)
            || (route_layout[route_index].start_page
                < route_layout[route_index].end_page)
            || nav_flash_page_is_in_route(NAV_FLASH_META_PAGE_A, route_index)
            || nav_flash_page_is_in_route(NAV_FLASH_META_PAGE_B, route_index)
            || (nav_flash_route_capacity(route_index)
                > NAV_FLASH_REPLAY_MAX_SAMPLES))
        {
            return 0;
        }

        for (other_index = (uint8)(route_index + 1U);
             other_index < NAV_FLASH_ROUTE_COUNT;
             other_index++)
        {
            // Ranges are inclusive and stored as high(start)..low(end).
            // Check both bounds so a smaller range nested inside a larger one
            // is also rejected, not only ranges whose endpoints cross.
            if ((route_layout[route_index].end_page
                 <= route_layout[other_index].start_page)
                && (route_layout[other_index].end_page
                    <= route_layout[route_index].start_page))
            {
                return 0;
            }
        }
    }
    return 1;
}

static uint32 nav_flash_crc32_words(const uint32 *data, uint32 word_count)
{
    uint32 crc = 0xFFFFFFFFUL;
    uint32 word_index;
    uint8 byte_index;
    uint8 bit_index;

    for (word_index = 0; word_index < word_count; word_index++)
    {
        uint32 word = data[word_index];

        for (byte_index = 0; byte_index < 4U; byte_index++)
        {
            crc ^= (word >> (8U * byte_index)) & 0xFFU;
            for (bit_index = 0; bit_index < 8U; bit_index++)
            {
                uint32 mask = (uint32)(0U - (crc & 1U));
                crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
            }
        }
    }
    return ~crc;
}

static void nav_flash_clear_page(uint32 *page)
{
    // FLASH_PAGE_LENGTH is a word count; sizeof is required to clear all 2048
    // bytes. The shared driver's flash_buffer_clear() clears only 512 bytes.
    memset(page, 0xFF, sizeof(uint32) * FLASH_PAGE_LENGTH);
}

static void nav_flash_reset_metadata(nav_flash_metadata_t *value)
{
    uint8 route_index;

    memset(value, 0, sizeof(*value));
    for (route_index = 0; route_index < NAV_FLASH_ROUTE_COUNT; route_index++)
    {
        value->route[route_index].start_page = route_layout[route_index].start_page;
        value->route[route_index].end_page = route_layout[route_index].end_page;
    }
}

static void nav_flash_serialize_metadata(const nav_flash_metadata_t *value,
                                         uint32 *page)
{
    uint8 route_index;

    nav_flash_clear_page(page);
    page[NAV_FLASH_META_MAGIC_WORD] = NAV_FLASH_METADATA_MAGIC;
    page[NAV_FLASH_META_VERSION_WORD] = NAV_FLASH_FORMAT_VERSION;
    page[NAV_FLASH_META_GENERATION_WORD] = value->generation;
    page[NAV_FLASH_META_ROUTE_COUNT_WORD] = NAV_FLASH_ROUTE_COUNT;

    for (route_index = 0; route_index < NAV_FLASH_ROUTE_COUNT; route_index++)
    {
        uint32 base = NAV_FLASH_META_ROUTE_BASE_WORD
                      + (uint32)route_index * NAV_FLASH_META_ROUTE_WORDS;
        const nav_flash_route_metadata_t *route = &value->route[route_index];

        page[base + NAV_FLASH_META_ROUTE_VALID_OFFSET] = route->valid;
        page[base + NAV_FLASH_META_ROUTE_START_OFFSET] = route->start_page;
        page[base + NAV_FLASH_META_ROUTE_END_OFFSET] = route->end_page;
        page[base + NAV_FLASH_META_ROUTE_COUNT_OFFSET] = route->sample_count;
        page[base + NAV_FLASH_META_ROUTE_GENERATION_OFFSET]
            = route->data_generation;
    }

    page[NAV_FLASH_META_CRC_WORD]
        = nav_flash_crc32_words(page, NAV_FLASH_META_CRC_INPUT_WORDS);
    page[NAV_FLASH_COMMIT_WORD] = NAV_FLASH_COMMIT_MARKER;
}

static uint8 nav_flash_parse_metadata(const uint32 *page,
                                      nav_flash_metadata_t *value)
{
    uint8 route_index;

    if ((page[NAV_FLASH_META_MAGIC_WORD] != NAV_FLASH_METADATA_MAGIC)
        || (page[NAV_FLASH_META_VERSION_WORD] != NAV_FLASH_FORMAT_VERSION)
        || (page[NAV_FLASH_META_ROUTE_COUNT_WORD] != NAV_FLASH_ROUTE_COUNT)
        || (page[NAV_FLASH_COMMIT_WORD] != NAV_FLASH_COMMIT_MARKER)
        || (page[NAV_FLASH_META_CRC_WORD]
            != nav_flash_crc32_words(page, NAV_FLASH_META_CRC_INPUT_WORDS)))
    {
        return 0;
    }

    nav_flash_reset_metadata(value);
    value->generation = page[NAV_FLASH_META_GENERATION_WORD];
    for (route_index = 0; route_index < NAV_FLASH_ROUTE_COUNT; route_index++)
    {
        uint32 base = NAV_FLASH_META_ROUTE_BASE_WORD
                      + (uint32)route_index * NAV_FLASH_META_ROUTE_WORDS;
        nav_flash_route_metadata_t *route = &value->route[route_index];

        route->valid = page[base + NAV_FLASH_META_ROUTE_VALID_OFFSET];
        route->start_page = page[base + NAV_FLASH_META_ROUTE_START_OFFSET];
        route->end_page = page[base + NAV_FLASH_META_ROUTE_END_OFFSET];
        route->sample_count = page[base + NAV_FLASH_META_ROUTE_COUNT_OFFSET];
        route->data_generation
            = page[base + NAV_FLASH_META_ROUTE_GENERATION_OFFSET];

        if ((route->valid > 1U)
            || (route->start_page != route_layout[route_index].start_page)
            || (route->end_page != route_layout[route_index].end_page)
            || (route->sample_count > nav_flash_route_capacity(route_index))
            || ((route->valid != 0U)
                && ((route->sample_count == 0U)
                    || (route->data_generation == 0U))))
        {
            return 0;
        }
    }
    return 1;
}

static nav_flash_status_t nav_flash_write_and_verify(uint8 page,
                                                      const uint32 *data)
{
    uint8 attempt;

    for (attempt = 0; attempt < NAV_FLASH_WRITE_RETRY_COUNT; attempt++)
    {
        flash_write_page(0, page, data, FLASH_PAGE_LENGTH);
#if NAV_FLASH_VERIFY_AFTER_WRITE
        flash_read_page(0, page, verify_buffer, FLASH_PAGE_LENGTH);
        if (0 == memcmp(data, verify_buffer, sizeof(verify_buffer)))
        {
            return NAV_FLASH_STATUS_OK;
        }
#else
        return NAV_FLASH_STATUS_OK;
#endif
    }
    return NAV_FLASH_STATUS_IO_ERROR;
}

static nav_flash_status_t nav_flash_commit_metadata(
    const nav_flash_metadata_t *new_metadata)
{
    uint8 target_page;
    nav_flash_status_t status;

    target_page = (active_metadata_page == NAV_FLASH_META_PAGE_A)
                  ? NAV_FLASH_META_PAGE_B : NAV_FLASH_META_PAGE_A;
    nav_flash_serialize_metadata(new_metadata, io_buffer);
    status = nav_flash_write_and_verify(target_page, io_buffer);
    if (NAV_FLASH_STATUS_OK == status)
    {
        metadata = *new_metadata;
        active_metadata_page = target_page;
    }
    return status;
}

static uint32 nav_flash_next_generation(void)
{
    uint32 generation = metadata.generation + 1U;

    // Generation zero is reserved for routes that have never been committed.
    if (0U == generation)
    {
        generation = 1U;
    }
    return generation;
}

static float nav_flash_wrap_yaw(float yaw_deg)
{
    while (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg <= -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

static int32 nav_flash_encode_yaw(float yaw_deg)
{
    float scaled = nav_flash_wrap_yaw(yaw_deg) * NAV_FLASH_YAW_SCALE;

    return (int32)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static void nav_flash_prepare_data_page(uint8 buffer_index,
                                        uint16 sample_count,
                                        uint16 page_sequence)
{
    uint32 *page = record_buffer[buffer_index];

    page[NAV_FLASH_DATA_MAGIC_WORD] = NAV_FLASH_DATA_MAGIC;
    page[NAV_FLASH_DATA_VERSION_WORD] = NAV_FLASH_FORMAT_VERSION;
    page[NAV_FLASH_DATA_ROUTE_WORD] = record_route_id;
    page[NAV_FLASH_DATA_SEQUENCE_WORD] = page_sequence;
    page[NAV_FLASH_DATA_COUNT_WORD] = sample_count;
    page[NAV_FLASH_DATA_GENERATION_WORD] = record_generation;
    page[NAV_FLASH_DATA_CRC_WORD]
        = nav_flash_crc32_words(page, NAV_FLASH_DATA_CRC_INPUT_WORDS);
    page[NAV_FLASH_COMMIT_WORD] = NAV_FLASH_COMMIT_MARKER;
}

static nav_flash_status_t nav_flash_queue_active_buffer(void)
{
    uint8 route_index = (uint8)(record_route_id - 1U);
    uint8 buffer_to_queue;

    if (0U == active_sample_count)
    {
        return NAV_FLASH_STATUS_OK;
    }
    if (pending_buffer != NAV_FLASH_NO_BUFFER)
    {
        return NAV_FLASH_STATUS_BUSY;
    }
    if (next_page_sequence >= nav_flash_route_page_count(route_index))
    {
        return NAV_FLASH_STATUS_FULL;
    }

    buffer_to_queue = active_record_buffer;
    nav_flash_prepare_data_page(buffer_to_queue,
                                active_sample_count,
                                next_page_sequence);
    pending_page = (uint8)(route_layout[route_index].start_page
                           - next_page_sequence);
    next_page_sequence++;

    active_record_buffer = (uint8)(1U - active_record_buffer);
    active_sample_count = 0;
    nav_flash_clear_page(record_buffer[active_record_buffer]);

    // Publish the buffer only after its footer and page number are complete.
    pending_buffer = buffer_to_queue;
    return NAV_FLASH_STATUS_OK;
}

static uint8 nav_flash_data_page_is_valid(const uint32 *page,
                                          uint8 route_id,
                                          uint16 page_sequence,
                                          uint16 expected_count,
                                          uint32 expected_generation)
{
    return (uint8)
    (
        (page[NAV_FLASH_DATA_MAGIC_WORD] == NAV_FLASH_DATA_MAGIC)
        && (page[NAV_FLASH_DATA_VERSION_WORD] == NAV_FLASH_FORMAT_VERSION)
        && (page[NAV_FLASH_DATA_ROUTE_WORD] == route_id)
        && (page[NAV_FLASH_DATA_SEQUENCE_WORD] == page_sequence)
        && (page[NAV_FLASH_DATA_COUNT_WORD] == expected_count)
        && (page[NAV_FLASH_DATA_GENERATION_WORD] == expected_generation)
        && (page[NAV_FLASH_DATA_CRC_WORD]
            == nav_flash_crc32_words(page, NAV_FLASH_DATA_CRC_INPUT_WORDS))
        && (page[NAV_FLASH_COMMIT_WORD] == NAV_FLASH_COMMIT_MARKER)
    );
}

nav_flash_status_t nav_flash_init(void)
{
    nav_flash_metadata_t metadata_a;
    nav_flash_metadata_t metadata_b;
    uint8 valid_a;
    uint8 valid_b;

    module_state = NAV_FLASH_STATE_UNINITIALIZED;
    last_status = NAV_FLASH_STATUS_NOT_INITIALIZED;
    active_metadata_page = NAV_FLASH_NO_PAGE;
    replay_sample_count = 0;
    replay_route_id = 0;

    if (!nav_flash_config_is_valid())
    {
        last_status = NAV_FLASH_STATUS_INVALID_CONFIG;
        module_state = NAV_FLASH_STATE_ERROR;
        return last_status;
    }

    flash_read_page(0, NAV_FLASH_META_PAGE_A, io_buffer, FLASH_PAGE_LENGTH);
    valid_a = nav_flash_parse_metadata(io_buffer, &metadata_a);
    flash_read_page(0, NAV_FLASH_META_PAGE_B, io_buffer, FLASH_PAGE_LENGTH);
    valid_b = nav_flash_parse_metadata(io_buffer, &metadata_b);

    if (valid_a && valid_b)
    {
        if ((int32)(metadata_a.generation - metadata_b.generation) >= 0)
        {
            metadata = metadata_a;
            active_metadata_page = NAV_FLASH_META_PAGE_A;
        }
        else
        {
            metadata = metadata_b;
            active_metadata_page = NAV_FLASH_META_PAGE_B;
        }
    }
    else if (valid_a)
    {
        metadata = metadata_a;
        active_metadata_page = NAV_FLASH_META_PAGE_A;
    }
    else if (valid_b)
    {
        metadata = metadata_b;
        active_metadata_page = NAV_FLASH_META_PAGE_B;
    }
    else
    {
        // A blank or old-format device starts with an empty in-RAM directory.
        // Metadata is written only after the first route is committed.
        nav_flash_reset_metadata(&metadata);
    }

    pending_buffer = NAV_FLASH_NO_BUFFER;
    module_state = NAV_FLASH_STATE_IDLE;
    last_status = NAV_FLASH_STATUS_OK;
    return last_status;
}

nav_flash_status_t nav_flash_record_start(uint8 route_id)
{
    if (NAV_FLASH_STATE_UNINITIALIZED == module_state)
    {
        return NAV_FLASH_STATUS_NOT_INITIALIZED;
    }
    if (!nav_flash_route_id_is_valid(route_id))
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }
    if ((NAV_FLASH_STATE_IDLE != module_state)
        && (NAV_FLASH_STATE_REPLAY_READY != module_state))
    {
        return NAV_FLASH_STATUS_BUSY;
    }

    nav_flash_clear_page(record_buffer[0]);
    nav_flash_clear_page(record_buffer[1]);
    active_record_buffer = 0;
    pending_buffer = NAV_FLASH_NO_BUFFER;
    active_sample_count = 0;
    next_page_sequence = 0;
    record_sample_count = 0;
    record_distance = 0.0f;
    record_route_id = route_id;
    record_generation = nav_flash_next_generation();
    finish_requested = 0;
    replay_sample_count = 0;
    replay_route_id = 0;
    last_status = NAV_FLASH_STATUS_OK;
    module_state = NAV_FLASH_STATE_RECORDING;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_record_sample(float yaw_deg,
                                            float distance_delta)
{
    uint8 route_index;

    if (NAV_FLASH_STATE_RECORDING != module_state)
    {
        return (NAV_FLASH_STATE_UNINITIALIZED == module_state)
               ? NAV_FLASH_STATUS_NOT_INITIALIZED : NAV_FLASH_STATUS_BUSY;
    }
    if ((yaw_deg != yaw_deg)
        || (distance_delta != distance_delta)
        || (yaw_deg > FLT_MAX)
        || (yaw_deg < -FLT_MAX)
        || (distance_delta < 0.0f)
        || (distance_delta > FLT_MAX))
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }

    route_index = (uint8)(record_route_id - 1U);
    record_distance += distance_delta;
    while (record_distance >= NAV_FLASH_SAMPLE_DISTANCE)
    {
        nav_flash_status_t status;

        if (record_sample_count >= nav_flash_route_capacity(route_index))
        {
            last_status = NAV_FLASH_STATUS_FULL;
            return last_status;
        }
        if (active_sample_count >= NAV_FLASH_SAMPLES_PER_PAGE)
        {
            status = nav_flash_queue_active_buffer();
            if (NAV_FLASH_STATUS_OK != status)
            {
                last_status = status;
                return status;
            }
        }

        record_buffer[active_record_buffer][active_sample_count]
            = (uint32)nav_flash_encode_yaw(yaw_deg);
        active_sample_count++;
        record_sample_count++;
        record_distance -= NAV_FLASH_SAMPLE_DISTANCE;

        if ((active_sample_count >= NAV_FLASH_SAMPLES_PER_PAGE)
            && (pending_buffer == NAV_FLASH_NO_BUFFER))
        {
            status = nav_flash_queue_active_buffer();
            if (NAV_FLASH_STATUS_OK != status)
            {
                last_status = status;
                return status;
            }
        }
    }

    last_status = NAV_FLASH_STATUS_OK;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_record_stop(void)
{
    nav_flash_status_t status;

    if (NAV_FLASH_STATE_RECORDING != module_state)
    {
        return (NAV_FLASH_STATE_UNINITIALIZED == module_state)
               ? NAV_FLASH_STATUS_NOT_INITIALIZED : NAV_FLASH_STATUS_BUSY;
    }
    if (0U == record_sample_count)
    {
        module_state = NAV_FLASH_STATE_IDLE;
        last_status = NAV_FLASH_STATUS_NOT_FOUND;
        return last_status;
    }

    // Stop accepting ISR samples before examining the active buffer.
    module_state = NAV_FLASH_STATE_FLUSH_PENDING;
    finish_requested = 1;
    if ((pending_buffer == NAV_FLASH_NO_BUFFER) && (active_sample_count != 0U))
    {
        status = nav_flash_queue_active_buffer();
        if (NAV_FLASH_STATUS_OK != status)
        {
            module_state = NAV_FLASH_STATE_ERROR;
            last_status = status;
            return status;
        }
    }

    last_status = NAV_FLASH_STATUS_OK;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_service(void)
{
    nav_flash_status_t status;

    if (NAV_FLASH_STATE_UNINITIALIZED == module_state)
    {
        return NAV_FLASH_STATUS_NOT_INITIALIZED;
    }
    if (NAV_FLASH_STATE_ERROR == module_state)
    {
        return last_status;
    }
    if ((NAV_FLASH_STATE_RECORDING != module_state)
        && (NAV_FLASH_STATE_FLUSH_PENDING != module_state))
    {
        return NAV_FLASH_STATUS_OK;
    }

    if (pending_buffer != NAV_FLASH_NO_BUFFER)
    {
        uint8 buffer_to_write = pending_buffer;

        status = nav_flash_write_and_verify(pending_page,
                                             record_buffer[buffer_to_write]);
        if (NAV_FLASH_STATUS_OK != status)
        {
            module_state = NAV_FLASH_STATE_ERROR;
            last_status = status;
            return status;
        }
        pending_buffer = NAV_FLASH_NO_BUFFER;
    }

    // If both RAM pages became full before service ran, queue the second page
    // now. It will be written by the next main-loop service call.
    if ((active_sample_count >= NAV_FLASH_SAMPLES_PER_PAGE)
        && (pending_buffer == NAV_FLASH_NO_BUFFER))
    {
        status = nav_flash_queue_active_buffer();
        if (NAV_FLASH_STATUS_OK != status)
        {
            module_state = NAV_FLASH_STATE_ERROR;
            last_status = status;
            return status;
        }
        last_status = NAV_FLASH_STATUS_OK;
        return NAV_FLASH_STATUS_OK;
    }

    if (finish_requested && (pending_buffer == NAV_FLASH_NO_BUFFER))
    {
        if (active_sample_count != 0U)
        {
            status = nav_flash_queue_active_buffer();
            if (NAV_FLASH_STATUS_OK != status)
            {
                module_state = NAV_FLASH_STATE_ERROR;
                last_status = status;
                return status;
            }
            return NAV_FLASH_STATUS_OK;
        }
        else
        {
            nav_flash_metadata_t new_metadata = metadata;
            uint8 route_index = (uint8)(record_route_id - 1U);

            new_metadata.generation = record_generation;
            new_metadata.route[route_index].valid = 1U;
            new_metadata.route[route_index].sample_count = record_sample_count;
            new_metadata.route[route_index].data_generation = record_generation;
            status = nav_flash_commit_metadata(&new_metadata);
            if (NAV_FLASH_STATUS_OK != status)
            {
                module_state = NAV_FLASH_STATE_ERROR;
                last_status = status;
                return status;
            }

            finish_requested = 0;
            record_route_id = 0;
            module_state = NAV_FLASH_STATE_IDLE;
        }
    }

    last_status = NAV_FLASH_STATUS_OK;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_load_route(uint8 route_id)
{
    uint8 route_index;
    uint16 page_sequence;
    uint32 remaining;
    uint32 destination_index = 0;
    const nav_flash_route_metadata_t *route;

    if (NAV_FLASH_STATE_UNINITIALIZED == module_state)
    {
        return NAV_FLASH_STATUS_NOT_INITIALIZED;
    }
    if (!nav_flash_route_id_is_valid(route_id))
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }
    if ((NAV_FLASH_STATE_IDLE != module_state)
        && (NAV_FLASH_STATE_REPLAY_READY != module_state))
    {
        return NAV_FLASH_STATUS_BUSY;
    }

    route_index = (uint8)(route_id - 1U);
    route = &metadata.route[route_index];
    if (!route->valid)
    {
        return NAV_FLASH_STATUS_NOT_FOUND;
    }
    if ((route->sample_count == 0U)
        || (route->sample_count > NAV_FLASH_REPLAY_MAX_SAMPLES)
        || (route->sample_count > nav_flash_route_capacity(route_index)))
    {
        return NAV_FLASH_STATUS_CORRUPT;
    }

    module_state = NAV_FLASH_STATE_LOADING;
    remaining = route->sample_count;
    page_sequence = 0;
    while (remaining != 0U)
    {
        uint16 page_sample_count = (uint16)
            ((remaining > NAV_FLASH_SAMPLES_PER_PAGE)
             ? NAV_FLASH_SAMPLES_PER_PAGE : remaining);
        uint8 page = (uint8)(route_layout[route_index].start_page
                             - page_sequence);
        uint16 sample_index;

        flash_read_page(0, page, io_buffer, FLASH_PAGE_LENGTH);
        if (!nav_flash_data_page_is_valid(io_buffer,
                                          route_id,
                                          page_sequence,
                                          page_sample_count,
                                          route->data_generation))
        {
            replay_sample_count = 0;
            replay_route_id = 0;
            module_state = NAV_FLASH_STATE_IDLE;
            last_status = NAV_FLASH_STATUS_CORRUPT;
            return last_status;
        }

        for (sample_index = 0; sample_index < page_sample_count; sample_index++)
        {
            replay_samples[destination_index++] = (int32)io_buffer[sample_index];
        }
        remaining -= page_sample_count;
        page_sequence++;
    }

    replay_route_id = route_id;
    replay_sample_count = route->sample_count;
    module_state = NAV_FLASH_STATE_REPLAY_READY;
    last_status = NAV_FLASH_STATUS_OK;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_get_sample(uint32 index, float *yaw_deg)
{
    if (0 == yaw_deg)
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }
    if (NAV_FLASH_STATE_REPLAY_READY != module_state)
    {
        return (NAV_FLASH_STATE_UNINITIALIZED == module_state)
               ? NAV_FLASH_STATUS_NOT_INITIALIZED : NAV_FLASH_STATUS_BUSY;
    }
    if (index >= replay_sample_count)
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }

    *yaw_deg = (float)replay_samples[index] / NAV_FLASH_YAW_SCALE;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_delete_route(uint8 route_id)
{
    uint8 route_index;
    nav_flash_metadata_t new_metadata;
    nav_flash_status_t status;

    if (NAV_FLASH_STATE_UNINITIALIZED == module_state)
    {
        return NAV_FLASH_STATUS_NOT_INITIALIZED;
    }
    if (!nav_flash_route_id_is_valid(route_id))
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }
    if ((NAV_FLASH_STATE_IDLE != module_state)
        && (NAV_FLASH_STATE_REPLAY_READY != module_state))
    {
        return NAV_FLASH_STATUS_BUSY;
    }

    route_index = (uint8)(route_id - 1U);
    if (!metadata.route[route_index].valid)
    {
        return NAV_FLASH_STATUS_NOT_FOUND;
    }

    new_metadata = metadata;
    new_metadata.generation = nav_flash_next_generation();
    new_metadata.route[route_index].valid = 0;
    new_metadata.route[route_index].sample_count = 0;
    new_metadata.route[route_index].data_generation = 0;
    status = nav_flash_commit_metadata(&new_metadata);
    if (NAV_FLASH_STATUS_OK != status)
    {
        module_state = NAV_FLASH_STATE_ERROR;
        last_status = status;
        return status;
    }

    if (replay_route_id == route_id)
    {
        replay_route_id = 0;
        replay_sample_count = 0;
        module_state = NAV_FLASH_STATE_IDLE;
    }
    last_status = NAV_FLASH_STATUS_OK;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_status_t nav_flash_get_route_info(
    uint8 route_id,
    nav_flash_route_info_t *info)
{
    uint8 route_index;

    if (NAV_FLASH_STATE_UNINITIALIZED == module_state)
    {
        return NAV_FLASH_STATUS_NOT_INITIALIZED;
    }
    if (!nav_flash_route_id_is_valid(route_id) || (0 == info))
    {
        return NAV_FLASH_STATUS_INVALID_ARGUMENT;
    }

    route_index = (uint8)(route_id - 1U);
    info->route_id = route_id;
    info->valid = (uint8)metadata.route[route_index].valid;
    info->reserved = 0;
    info->sample_count = metadata.route[route_index].sample_count;
    info->data_generation = metadata.route[route_index].data_generation;
    return NAV_FLASH_STATUS_OK;
}

nav_flash_state_t nav_flash_get_state(void)
{
    return module_state;
}

nav_flash_status_t nav_flash_get_last_status(void)
{
    return last_status;
}

uint32 nav_flash_get_loaded_sample_count(void)
{
    return replay_sample_count;
}
