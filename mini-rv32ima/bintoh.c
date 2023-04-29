#include <stdio.h>

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <dtb_file>\n", argv[0]);
        return -1;
    }

    FILE* file = fopen(argv[1], "rb");
    if (file == NULL) {
        printf("Failed to open file: %s\n", argv[1]);
        return -1;
    }

    int c, p = 0;
    printf("static const unsigned char default64mbdtb[] = {");
    while ((c = fgetc(file)) != EOF) {
        printf("0x%02x,%c", c, (((p++) & 15) == 15) ? 10 : ' ');
    }
    printf("};");

    fclose(file);
    return 0;
}
