A biblioteca do ICM20948 apresenta um erro no icm20948_spi.c
Por mais que não seja utilizada no código, o VSCode pode relatar esse e dar problema na hora de buildar.
Para corriginar, basta ir no managed_components e corrigir.
O erro tá nas linhas 21 e 45. Apenas substitua a linha por: "if (spi_device_polling_transmit(handle, &trans_desc) != ESP_OK)"
