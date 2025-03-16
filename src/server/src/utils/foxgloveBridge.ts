import { FoxgloveServer } from '@foxglove/ws-protocol';
import { TopicData } from "./";

const Channels = new Map<string, number>();
const textEncoder = new TextEncoder();

const sendToFoxglove = async (server: FoxgloveServer, message: TopicData | any) => {
  let data: TopicData;
  
  if (Buffer.isBuffer(message)) {
      // Если message является буфером, преобразуем его в строку и парсим
      data = JSON.parse(message.toString());
  } else if (typeof message === 'string') {
      // Если message является строкой, парсим её
      data = JSON.parse(message);
  } else if (typeof message === 'object' && message !== null) {
      // Если message уже является объектом, используем его
      data = message;
  } else {
      throw new Error('Invalid message format');
  }

  if (!Channels.has(data.topic)) {
     const  generateSchema = (obj: Record<string, any>): any => {
      const schema: any = { type: "object", properties: {} };
      for (const key in obj) {
        if (Array.isArray(obj[key])) {
          if (obj[key].length > 0 && typeof obj[key][0] === 'object') {
            schema.properties[key] = {
              type: "array",
              items: generateSchema(obj[key][0]),
            };
          } else {
            schema.properties[key] = {
              type: "array",
              items: { type: typeof obj[key][0] === 'number' ? 'number' : 'string' },
            };
          }
        } else if (typeof obj[key] === 'object' && obj[key] !== null) {
          schema.properties[key] = generateSchema(obj[key]);
        } else if (obj[key] === null || obj[key] === undefined) {
          schema.properties[key] = { type: 'string' };
        } else if (Number.isNaN(obj[key]) || !Number.isFinite(obj[key])) {
          schema.properties[key] = { type: 'string' };
        } else {
          schema.properties[key] = { type: typeof obj[key] === 'number' ? 'number' : 'string' };
        }
      }
      return schema;
    };

    const schema = generateSchema(data.data);

    const channelId = server.addChannel({
      topic: data.name,
      encoding: "json",
      schemaName: data.topic,
      schema: JSON.stringify(schema),
    });

    Channels.set(data.topic, channelId);
  }

  await server.sendMessage(
    Channels.get(data.topic)!,
    BigInt(data.timestamp),
    textEncoder.encode(JSON.stringify(data.data, (_, v) => typeof v === 'bigint' ? v.toString() : v)),
  );
};


export default sendToFoxglove;
