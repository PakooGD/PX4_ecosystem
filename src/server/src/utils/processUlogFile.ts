import { MessageType, ULog } from "@foxglove/ulog";
import { FileReader } from "@foxglove/ulog/node";
import { formatString,sendToFoxglove,delay  } from "./";
import { FoxgloveServer } from '@foxglove/ws-protocol';

export interface TopicData {
    name: string;
    topic: string;
    timestamp: bigint;
    data: Record<string, any>;
}

export async function processUlogFile(server: FoxgloveServer, filePath: string) {
    try {
        const ulog = new ULog(new FileReader(filePath));
        await ulog.open(); 
        for await (const msg of ulog.readMessages()) {
            if (msg.type === MessageType.Data) {
                const dataTopic = String(ulog.subscriptions.get(msg.msgId)?.name)
                const name = formatString(dataTopic)

                const message: TopicData = {
                    name: name,
                    topic: `ulog/${dataTopic}`,
                    timestamp: BigInt(msg.value.timestamp),
                    data: msg.value,
                };
                await sendToFoxglove(server, message);
                await delay(0.005); // Задержка 5 мс обязательна для отображения данных
            }
        }
        console.log("Messages processed and stored in Map.");
    } catch (error) {
        console.error("Error processing ULog file:", error);
    } finally {
    //    FileReader.prototype.close()
    }
}

