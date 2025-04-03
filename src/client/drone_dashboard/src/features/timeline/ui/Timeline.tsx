import React, { useState, useEffect } from 'react';
import { observer } from 'mobx-react-lite';
import { droneStore } from '../../../shared/stores/droneStore'
import styles from './Timeline.module.scss';
import { formatValue } from '../../droneLog/lib/formatValue';

interface TimelineProps {
    droneId: string;
    topic: string;
}

export const Timeline: React.FC<TimelineProps> = observer(({ droneId, topic }) => {
    const [selectedTimestampIndex, setSelectedTimestampIndex] = useState<number>(0);
    const [timestamps, setTimestamps] = useState<string[]>([]);

    // Получаем все логи для данного топика
    const topicLogs = droneStore.logs.get(droneId)?.get(topic) || [];

    // Обновляем список временных меток при изменении логов
    useEffect(() => {
        const newTimestamps = topicLogs.map((log) => log.timestamp);
        setTimestamps(newTimestamps);
        setSelectedTimestampIndex(newTimestamps.length - 1); // По умолчанию выбираем последний лог
    }, [topicLogs]);

    // Обработчик изменения позиции ползунка
    const handleSliderChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        const index = parseInt(event.target.value, 10);
        setSelectedTimestampIndex(index);
    };

    // Получаем данные на выбранный момент времени
    const selectedLog = topicLogs[selectedTimestampIndex];

    return (
        <div className={styles.timelineContainer}>
            <h3>Timeline for {topic}</h3>
            <div className={styles.timelineSlider}>
                <input
                    type="range"
                    min="0"
                    max={timestamps.length - 1}
                    value={selectedTimestampIndex}
                    onChange={handleSliderChange}
                />
            </div>
            <div className={styles.selectedTime}>
                <strong>Selected Time:</strong> {selectedLog?.timestamp || 'No data'}
            </div>
            {selectedLog ? (
                <div className={styles.dataContainer}>
                    <h4>Data at {selectedLog.timestamp}</h4>
                    <div>
                        {Object.entries(selectedLog.data.data).map(([key, value]) => (
                            <div key={key} className={styles.dataItem}>
                                <strong>{key}:</strong> {formatValue(value)}
                            </div>
                        ))}
                    </div>
                </div>
            ) : (
                <p className={styles.noData}>No data available for selected timestamp.</p>
            )}
        </div>
    );
});