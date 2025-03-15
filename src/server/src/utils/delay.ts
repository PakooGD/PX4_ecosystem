export function delay(durationSec:any) {
    return new Promise((resolve) => setTimeout(resolve, durationSec * 1000));
}