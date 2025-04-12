import { OperationFailed } from '../utils/errors/errors';
import fs from 'fs';
import path from 'path';
import { formatDate } from '../utils/helpers/FormatHelper';
import zlib from 'zlib'
import { ULogProcessor } from '../utils/helpers/LogHelper'
import { Json } from 'sequelize/types/utils';

const LOG_DIR = path.join(__dirname, '../../temp/ulog');
if (!fs.existsSync(LOG_DIR)) {
    fs.mkdirSync(LOG_DIR, { recursive: true });
}

let fileStream: fs.WriteStream | null = null;
let ulogProcessor: ULogProcessor | null = null;
let currentFile: {
    name: string,
    size: number,
    chunks: number,
    receivedChunks: number,
    data: Buffer[]
} | null = null;

export class LogService {
    public static async CloseStream(): Promise<void> {
        try {
            if (fileStream) {
                if (ulogProcessor) {
                    ulogProcessor.processStreamedUlogData([], 0, 0);
                }
                await new Promise((resolve) => {
                    fileStream!.end(resolve);
                });
            }
            
            fileStream = null;
            ulogProcessor = null;
        } catch (err) {
            throw new OperationFailed(`Operation failed: ${err}`);
        }
    }

    public static MavLogStream(decryptedData: any, droneId:string): void {
        try {
            if(!fileStream){
                const file = `${droneId}_${formatDate(Date.now(), "DD_MM_YYYY-HH_mm")}.ulg`
                const filename = path.join(LOG_DIR, file);
                fileStream = fs.createWriteStream(filename);
                ulogProcessor = new ULogProcessor({
                    write: (data: Uint8Array) => {
                        fileStream!.write(data);
                    }
                });
            }
            const binaryData = Array.from(Buffer.from(decryptedData.data, 'hex'));
            ulogProcessor?.processStreamedUlogData(
                binaryData,
                decryptedData.firstMsgStart,
                decryptedData.numDrops
            );
        } catch (err) {
            throw new OperationFailed(`Operation failed: ${err}`);
        }
    }

    public static LogReciever(decryptedData: any) {
        try {
            if (!currentFile) {
                const meta = JSON.parse(decryptedData.metadata)   
                currentFile = {
                    name: meta.name,
                    size: meta.size,
                    chunks: meta.chunks,
                    receivedChunks: 0,
                    data: []
                };
            } 

            const chunkData = Buffer.from(decryptedData.data, 'base64');
            const decompressed = zlib.inflateSync(chunkData);
            
            if (zlib.crc32(chunkData) !== decryptedData.checksum) {
                throw new Error('Checksum mismatch');
            }
            
            currentFile.data.push(decompressed);
            currentFile.receivedChunks++;

            if (decryptedData.finished || currentFile.receivedChunks === currentFile.chunks) {
                const fullData = Buffer.concat(currentFile.data);
                if (fullData.length !== currentFile.size) throw new Error('File size mismatch');
                fs.writeFileSync(path.join(LOG_DIR, currentFile.name), fullData);
                currentFile = null;  
            }
        } catch(err) {
            throw new OperationFailed(`Operation failed: ${err}`);
        }
    }
}


