# Procedure for SDIO SAPI Execution

## Steps

1. Navigate to the driver source directory:
    ```sh
    cd linux-sdio-driver/platforms/linux/Driver/sdio/src
    ```

2. Compile the driver:
    ```sh
    make clean
    make
    ```

3. Insert the driver:
    ```sh
    insmod rpssdio.ko
    ```

