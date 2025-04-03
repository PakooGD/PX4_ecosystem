export const formatValue = (value: any): React.ReactNode => {
    if (Array.isArray(value)) {
        // Если значение — массив, отображаем его элементы через запятую
        return value.join(', ');
    } else if (typeof value === 'object' && value !== null) {
        // Если значение — объект, отображаем его ключи и значения
        return (
            <div style={{ marginLeft: '20px' }}>
                {Object.entries(value).map(([subKey, subValue]) => (
                    <div key={subKey}>
                        <strong>{subKey}:</strong> {formatValue(subValue)}
                    </div>
                ))}
            </div>
        );
    } else {
        // Если значение — примитив, отображаем его как есть
        return value;
    }
};