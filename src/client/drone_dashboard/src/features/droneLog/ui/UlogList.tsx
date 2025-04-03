import React, { useState, useEffect } from 'react';
import { observer } from 'mobx-react-lite';
import { Dropdown } from '../../../shared/ui/Dropdown/Dropdown';
import { formatValue } from '../lib/formatValue';
import { ULogData, TopicData } from '../../../shared/types/ITypes';
import styles from '../../timeline/ui/Timeline.module.scss';

interface UlogListProps {
    droneId: string;
    ulog: ULogData;
}

export const UlogList: React.FC<UlogListProps> = observer(({ droneId, ulog }) => {
    const [selectedTimestampIndex, setSelectedTimestampIndex] = useState<number>(0);
    const [timestamps, setTimestamps] = useState<string[]>([]); // Изменено на string[]
    const [topicsData, setTopicsData] = useState<{ [topic: string]: any[] }>({});

    // Преобразуем данные ulog в удобный формат
    useEffect(() => {
        const allTimestamps: string[] = []; // Изменено на string[]
        const topics: { [topic: string]: TopicData[] } = {};

        // Проходим по всем топикам и собираем временные метки
        Object.entries(ulog.content).forEach(([topic, logs]) => {
            topics[topic] = logs;
            logs.forEach((log: any) => {
                // Преобразуем timestamp в строку
                const timestamp = log.timestamp.toString(); // Преобразуем в строку
                if (!allTimestamps.includes(timestamp)) {
                    allTimestamps.push(timestamp);
                }
            });
        });

        // Убираем сортировку, так как данные уже в правильном порядке
        setTimestamps(allTimestamps);
        setTopicsData(topics);
        setSelectedTimestampIndex(allTimestamps.length - 1); // По умолчанию выбираем последний лог
    }, [ulog]);

    // Обработчик изменения позиции ползунка
    const handleSliderChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        const index = parseInt(event.target.value, 10);
        setSelectedTimestampIndex(index);
    };

    // Получаем выбранный timestamp
    const selectedTimestamp = timestamps[selectedTimestampIndex];

    return (
        <Dropdown key={`ulog_${droneId}`} title={`${`ULog File: ${ulog.source}`}`}>
            <div className={styles.timelineContainer}>
                <h3>Timeline for {ulog.source}</h3>
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
                    <strong>Selected Time:</strong> {selectedTimestamp || 'No data'}
                </div>

                {/* Отображаем данные для каждого топика на выбранный момент времени */}
                {Object.entries(topicsData).map(([topic, logs]) => {
                    // Находим лог для выбранного времени
                    const selectedLog = logs.find(log => log.timestamp.toString() === selectedTimestamp); // Сравниваем как строки

                    return (
                        <Dropdown key={`${topic}_${droneId}`} title={`Topic: ${topic}`}>
                            <div className={styles.dataContainer}>
                                {selectedLog ? (
                                    <>
                                        <h4>Data at {selectedLog.timestamp}</h4>
                                        <div>
                                            {Object.entries(selectedLog.data).map(([key, value]) => (
                                                <div key={key} className={styles.dataItem}>
                                                    <strong>{key}:</strong> {formatValue(value)}
                                                </div>
                                            ))}
                                        </div>
                                    </>
                                ) : (
                                    <p className={styles.noData}>No data available for selected timestamp.</p>
                                )}
                            </div>
                        </Dropdown>
                    );
                })}
            </div>
        </Dropdown>
    );
});