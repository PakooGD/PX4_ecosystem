export const collectValues = (data: any, prefix = ''): { key: string; value: number }[] => {
    if (Array.isArray(data)) {
        return data.flatMap((item, index) => collectValues(item, `${prefix}[${index}]`));
    } else if (typeof data === 'object' && data !== null) {
        return Object.entries(data).flatMap(([key, value]) =>
            collectValues(value, prefix ? `${prefix}.${key}` : key),
        );
    } else if (typeof data === 'number') {
        return [{ key: prefix, value: data }];
    } else {
        return [];
    }
};