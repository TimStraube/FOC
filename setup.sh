#!/bin/bash

# Farben für Ausgaben
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# Überprüfe ob die notwendigen Tools installiert sind
echo "Überprüfe benötigte Tools..."

command -v arm-none-eabi-gcc >/dev/null 2>&1 || { 
    echo -e "${RED}arm-none-eabi-gcc nicht gefunden. Bitte installieren Sie die ARM GNU Toolchain.${NC}"
    read -p "Möchten Sie gcc-arm-none-eabi installieren? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get install gcc-arm-none-eabi
    else
        exit 1
    fi
}

command -v make >/dev/null 2>&1 || {
    echo -e "${RED}make nicht gefunden. Bitte installieren Sie make.${NC}"
    read -p "Möchten Sie make installieren? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get install make
    else
        exit 1
    fi
}

# Überprüfe zusätzlich benötigte Pakete für Ubuntu
command -v arm-none-eabi-objcopy >/dev/null 2>&1 || {
    echo -e "${RED}arm-none-eabi-binutils nicht gefunden.${NC}"
    read -p "Möchten Sie binutils-arm-none-eabi installieren? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get install binutils-arm-none-eabi
    else
        exit 1
    fi
}

# Überprüfe ob ST-Link Tools installiert sind
command -v st-flash >/dev/null 2>&1 || { 
    echo -e "${RED}st-flash nicht gefunden. Bitte installieren Sie die ST-Link Tools.${NC}"
    read -p "Möchten Sie stlink-tools installieren? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get install stlink-tools
    else
        exit 1
    fi
}

# Erstelle Build Verzeichnis
echo "Erstelle Build Verzeichnis..."
mkdir -p build

# Kompiliere Quellcode
echo "Kompiliere Projekt..."

arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb src/main.c -o build/main.o
if [ $? -ne 0 ]; then
    echo -e "${RED}Fehler beim Kompilieren von main.c${NC}"
    exit 1
fi

arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb src/startup.c -o build/startup.o
if [ $? -ne 0 ]; then
    echo -e "${RED}Fehler beim Kompilieren von startup.c${NC}"
    exit 1
fi

# Linken
echo "Linke Objekt-Dateien..."
arm-none-eabi-gcc -nostdlib -T src/linker_script.ld build/main.o build/startup.o -o build/foc.elf
if [ $? -ne 0 ]; then
    echo -e "${RED}Fehler beim Linken${NC}"
    exit 1
fi

# Erstelle Binary
arm-none-eabi-objcopy -O binary build/foc.elf build/foc.bin

echo -e "${GREEN}Build erfolgreich abgeschlossen!${NC}"
echo "Die Binary befindet sich in: build/foc.bin"

# Flashe das Programm
echo -e "${YELLOW}Verbinde mit ST-Link...${NC}"
st-flash --connect-under-reset write build/foc.bin 0x08000000
if [ $? -ne 0 ]; then
    echo -e "${RED}Fehler beim Flashen des Programms${NC}"
    echo "Bitte überprüfen Sie:"
    echo "1. Ist der ST-Link korrekt angeschlossen?"
    echo "2. Ist das Board eingeschaltet?"
    exit 1
fi

echo -e "${GREEN}Programm erfolgreich geflasht!${NC}"