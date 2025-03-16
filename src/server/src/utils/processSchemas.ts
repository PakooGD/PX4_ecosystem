import { FoxgloveServer } from '@foxglove/ws-protocol';

export const Channels = new Map<string, number>();
const textEncoder = new TextEncoder();

export interface topicSchema {
    topic: string,
    encoding: string,
    schemaName: string,
    schema: any,
}

export const processSchemas = async (server: FoxgloveServer, topicSchema: topicSchema) => {

    const channelId = server.addChannel({
      topic: topicSchema.topic,
      encoding: topicSchema.encoding,
      schemaName: topicSchema.schemaName,
      schema: JSON.stringify(topicSchema.schema),
    });

    Channels.set(topicSchema.schemaName, channelId);

  return channelId; 
};

export const getSchemaNameByChannelId = (chanId: number | string) => {
    return Array.from(Channels.entries()).find(([_, id]) => id === chanId)?.[0]
}
