export function formatString(input:string) {
    return input
        .split('_') // Разделить строку по символу '_'
        .map(word => word.charAt(0).toUpperCase() + word.slice(1)) // Каждое слово с заглавной буквы
        .join(''); // Объединить слова в одну строку
}