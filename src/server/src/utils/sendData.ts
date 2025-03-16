import { FoxgloveServer } from '@foxglove/ws-protocol';
import { Channels } from './';
import { TopicData } from './';
import { timeStamp } from 'console';

const textEncoder = new TextEncoder();

const sendData = async (server: FoxgloveServer, message: TopicData | any) => {
    let data: TopicData;
  
    if (Buffer.isBuffer(message) || typeof message === 'string') {
        data = JSON.parse(message.toString());
    } else if (typeof message === 'object' && message !== null) {
        data = message;
    } else {
        throw new Error('Invalid message format');
    }

    await server.sendMessage(
        Channels.get(data.topic)!,
        BigInt(data.timestamp),
        textEncoder.encode(JSON.stringify(data.data, (_, v) => typeof v === 'bigint' ? v.toString() : v)),
    );
};

export default sendData;
