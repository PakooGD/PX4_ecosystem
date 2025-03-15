export const hexToJSON = (data:any) => {
    // Если данные приходят в виде буфера (hex), преобразуем их в JSON
    if (Buffer.isBuffer(data)) {
        const hexString = data.toString('hex'); // Преобразуем буфер в шестнадцатеричную строку
        const jsonString = Buffer.from(hexString, 'hex').toString('utf-8'); // Преобразуем hex в строку UTF-8
        try {
            const jsonData = JSON.parse(jsonString); // Парсим строку в JSON
            return jsonData; 
        } catch (error) {
            console.error('❌ Failed to parse JSON:', error);
            return error;     
        }
    }
}

