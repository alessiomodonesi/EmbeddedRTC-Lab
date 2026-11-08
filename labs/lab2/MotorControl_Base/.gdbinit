# Crea un file chiamato .gdbinit nella root del progetto con questo contenuto

# Connessione automatica a OpenOCD
target remote localhost:3333

# Carica il firmware sull'MCU
load

# Azzera il programma
monitor reset halt

# Imposta un breakpoint all'inizio del main
break main

# Continua l'esecuzione dopo il reset
continue
