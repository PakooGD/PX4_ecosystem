import React from 'react';
import { observer } from 'mobx-react-lite';
import { DroneTopicBlock } from './DroneTopicBlock';
import { Log, ULogData } from '../../../shared/types/ITypes';
import { Dropdown } from '../../../shared/ui/Dropdown/Dropdown';
import { UlogList } from './UlogList';

interface DroneTopicListProps {
    droneId: string;
    topics?: Map<string, Log[]>; 
    ulog?: ULogData;
}

export const DroneTopicList: React.FC<DroneTopicListProps> = observer(({ droneId, topics, ulog }) => {
    return (
        <Dropdown title={`Drone ID: ${droneId}`}>
            {/* Отображаем блоки топиков, если topics существует */}
            {topics && Array.from(topics.entries()).map(([topic, logs]) => (
                <DroneTopicBlock key={`${droneId}_${topic}`} drone={droneId} topic={topic} logs={logs} />
            ))}

            {/* Отображаем UlogList, если ulog существует */}
            {ulog && (
                <UlogList
                    key={`${droneId}_ulog`} // Уникальный ключ
                    droneId={droneId}
                    ulog={ulog}
                />
            )}
        </Dropdown>
    );
});