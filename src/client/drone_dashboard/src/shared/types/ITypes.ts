export interface AuthResponse {
    accessToken:string;
    refreshToken:string;
    drone: Drone;
}

export interface DroneData {
    id: string;
    topics: TopicSchema[];
    status: 'online' | 'offline';
    ip_address: string;
}

export interface Drone{
    id:string;
    topics:TopicSchema[];
}
  
export interface Topic {
    name: string;
    isSubscribed: boolean;
}

export interface TopicSchema {
    name: string,
    topic: string,
    status: boolean
}

export interface TopicStatus {
    name: string;
    topic: string;
    status: boolean;
}
  
export interface Log {
    id: string;
    timestamp: string;
    data: TopicData;
}

export interface TopicData {
    name: string;
    topic: string;
    timestamp: bigint;
    data: Record<string, any>;
}



export interface LogMessage {
    droneId: string;
    content: TopicData;
}

export interface ULogData {
    source: string;
    content: Record<string, TopicData[]>;
}