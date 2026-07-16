#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Cone_vision.h"
#include "Minefield_vision.h"
#include "Terrain_vision.h"
#include "Vision_frame.h"
#include "zf_device_mt9v03x.h"

volatile uint8 mt9v03x_finish_flag;
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];

uint8 mt9v03x_set_exposure_time(uint16 light)
{
    (void)light;
    return 0U;
}

uint8 mt9v03x_init(void)
{
    return 0U;
}

static int read_header_token(FILE *file, char *token, size_t capacity)
{
    size_t length = 0U;
    int character;

    do
    {
        character = fgetc(file);
        if ('#' == character)
        {
            do
            {
                character = fgetc(file);
            }
            while ((EOF != character) && ('\n' != character));
        }
    }
    while ((EOF != character) && isspace((unsigned char)character));
    if (EOF == character)
    {
        return 0;
    }

    while ((EOF != character) && !isspace((unsigned char)character)
           && ('#' != character))
    {
        if (length + 1U >= capacity)
        {
            return 0;
        }
        token[length++] = (char)character;
        character = fgetc(file);
    }
    token[length] = '\0';
    if ('\r' == character)
    {
        character = fgetc(file);
        if ((EOF != character) && ('\n' != character))
        {
            (void)ungetc(character, file);
        }
    }
    return (length > 0U) ? 1 : 0;
}

static uint8 *read_pgm(const char *path, uint16 *width, uint16 *height)
{
    char token[32];
    unsigned long parsed_width;
    unsigned long parsed_height;
    unsigned long maximum_gray;
    size_t pixel_count;
    uint8 *pixels;
    FILE *file = fopen(path, "rb");

    if (0 == file)
    {
        return 0;
    }
    if (!read_header_token(file, token, sizeof(token))
        || (0 != strcmp(token, "P5"))
        || !read_header_token(file, token, sizeof(token)))
    {
        (void)fclose(file);
        return 0;
    }
    parsed_width = strtoul(token, 0, 10);
    if (!read_header_token(file, token, sizeof(token)))
    {
        (void)fclose(file);
        return 0;
    }
    parsed_height = strtoul(token, 0, 10);
    if (!read_header_token(file, token, sizeof(token)))
    {
        (void)fclose(file);
        return 0;
    }
    maximum_gray = strtoul(token, 0, 10);
    if ((0U == parsed_width) || (parsed_width > 65535U)
        || (0U == parsed_height) || (parsed_height > 65535U)
        || (0U == maximum_gray) || (maximum_gray > 255U)
        || (parsed_width * parsed_height > 16000000U))
    {
        (void)fclose(file);
        return 0;
    }

    pixel_count = (size_t)parsed_width * parsed_height;
    pixels = (uint8 *)malloc(pixel_count);
    if ((0 == pixels) || (pixel_count != fread(pixels, 1U, pixel_count, file)))
    {
        free(pixels);
        (void)fclose(file);
        return 0;
    }
    (void)fclose(file);
    if (maximum_gray < 255U)
    {
        size_t index;

        for (index = 0U; index < pixel_count; index++)
        {
            pixels[index] = (uint8)((unsigned long)pixels[index] * 255U
                                    / maximum_gray);
        }
    }
    *width = (uint16)parsed_width;
    *height = (uint16)parsed_height;
    return pixels;
}

static const char *terrain_name(terrain_type_t type)
{
    static const char *const name[] = {
        "normal", "step", "bridge", "bumpy", "obstacle", "lost"
    };

    return ((uint32)type < (sizeof(name) / sizeof(name[0])))
        ? name[type] : "unknown";
}

int main(int argument_count, char **argument_value)
{
    int index;

    if (argument_count < 2)
    {
        (void)fprintf(stderr, "usage: %s frame1.pgm [frame2.pgm ...]\n",
                      argument_value[0]);
        return 2;
    }
    if (TERRAIN_VISION_STATUS_OK != terrain_vision_init()
        || (CONE_VISION_STATUS_WAITING_FOR_FRAME != cone_vision_init())
        || (MINEFIELD_VISION_STATUS_WAITING_FOR_FRAME
            != minefield_vision_init()))
    {
        (void)fprintf(stderr, "vision observer initialization failed\n");
        return 2;
    }

    puts("source\tgray\tterrain\tterrain_conf\tpath_valid\tpath_center"
         "\tpath_heading\tcone_count\tgap_valid\tgap_center"
         "\tmine_frame\tmine_center\tboundary_row\tboundary_warning");
    for (index = 1; index < argument_count; index++)
    {
        vision_frame_t frame;
        terrain_vision_result_t terrain;
        cone_vision_result_t cone;
        minefield_vision_result_t mine;
        uint16 width;
        uint16 height;
        uint8 *pixels = read_pgm(argument_value[index], &width, &height);

        if (0 == pixels)
        {
            (void)fprintf(stderr, "%s: invalid 8-bit binary PGM\n",
                          argument_value[index]);
            continue;
        }
        memset(&frame, 0, sizeof(frame));
        frame.status = VISION_FRAME_STATUS_OK;
        frame.enabled = 1U;
        frame.frame_count = (uint32)index;
        if (VISION_FRAME_STATUS_OK
            != vision_frame_resample(pixels,
                                     width,
                                     height,
                                     frame.gray[0],
                                     &frame.average_gray))
        {
            free(pixels);
            continue;
        }
        (void)terrain_vision_process_frame(pixels, width, height);
        (void)cone_vision_process_frame(&frame);
        (void)minefield_vision_process_frame(&frame);
        (void)terrain_vision_get_snapshot(&terrain);
        (void)cone_vision_get_snapshot(&cone);
        (void)minefield_vision_get_snapshot(&mine);
        printf("%s\t%u\t%s\t%u\t%u\t%.5f\t%.5f\t%u\t%u\t%.5f"
               "\t%u\t%.5f\t%u\t%u\n",
               argument_value[index],
               frame.average_gray,
               terrain_name(terrain.type),
               terrain.confidence,
               terrain.path_valid,
               terrain.path_center_error_norm,
               terrain.path_heading_error_norm,
               cone.candidate_count,
               cone.gap_valid,
               cone.gap_center_error_norm,
               mine.frame_candidate,
               mine.center_error_norm,
               mine.near_boundary_row_px,
               mine.boundary_warning);
        free(pixels);
    }
    return 0;
}
