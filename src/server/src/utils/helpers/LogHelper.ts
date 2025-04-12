
export class ULogProcessor {
    private gotUlogHeader: boolean = false;
    private gotHeaderSection: boolean = false;
    private ulogMessage: number[] = [];
    private file: { write: (data: Uint8Array) => void };

    constructor(file: { write: (data: Uint8Array) => void }) {
        this.file = file;
    }

    processStreamedUlogData(data: number[], firstMsgStart: number, numDrops: number): void {
        /** write streamed data to a file */
        if (!this.gotUlogHeader) { // the first 16 bytes need special treatment
            if (data.length < 16) { // that's never the case anyway
                throw new Error('first received message too short');
            }
            console.log("ULog header (first 16 bytes):", data.slice(0, 16));
            this.file.write(new Uint8Array(data.slice(0, 16)));
            data = data.slice(16);
            this.gotUlogHeader = true;
        }

        if (this.gotHeaderSection && numDrops > 0) {
            const adjustedNumDrops = numDrops > 25 ? 25 : numDrops;
            // write a dropout message. We don't really know the actual duration,
            // so just use the number of drops * 10 ms
            this.file.write(new Uint8Array([2, 0, 79, adjustedNumDrops * 10, 0]));
        }

        if (numDrops > 0) {
            this.writeUlogMessages(this.ulogMessage);
            this.ulogMessage = [];
            if (firstMsgStart === 255) {
                return; // no useful information in this message: drop it
            }
            data = data.slice(firstMsgStart);
            firstMsgStart = 0;
        }

        if (firstMsgStart === 255 && this.ulogMessage.length > 0) {
            this.ulogMessage.push(...data);
            return;
        }

        if (this.ulogMessage.length > 0) {
            this.file.write(new Uint8Array([...this.ulogMessage, ...data.slice(0, firstMsgStart)]));
            this.ulogMessage = [];
        }

        const remainingData = this.writeUlogMessages(data.slice(firstMsgStart));
        this.ulogMessage = remainingData; // store the rest for the next message
    }

    writeUlogMessages(data: number[]): number[] {
        /** write ulog data w/o integrity checking, assuming data starts with a
        valid ulog message. returns the remaining data at the end. */
        while (data.length > 2) {
            const messageLength = data[0] + data[1] * 256 + 3; // 3=ULog msg header
            if (messageLength > data.length) {
                break;
            }
            this.file.write(new Uint8Array(data.slice(0, messageLength)));
            data = data.slice(messageLength);
        }
        return data;
    }
}