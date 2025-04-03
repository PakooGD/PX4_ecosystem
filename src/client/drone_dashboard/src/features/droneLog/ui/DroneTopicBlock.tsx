import React, { useState, useMemo, useEffect } from 'react';
import { observer } from 'mobx-react-lite';
import { Dropdown } from '../../../shared/ui/Dropdown/Dropdown';
import { Timeline } from '../../timeline/ui/Timeline';
import { formatValue } from '../lib/formatValue';
import { Log } from '../../../shared/types/ITypes';
import { LineChartComponent } from '../../../shared/ui/LineChartComponent/LineChartComponent';
import { collectValues } from '../lib/collectValues';
import Slider from 'rc-slider';
import 'rc-slider/assets/index.css';

interface DroneTopicBlockProps {
    topic: string;
    drone: string;
    logs: Log[];
}

export const DroneTopicBlock: React.FC<DroneTopicBlockProps> = observer(({ topic, drone, logs }) => {
    const [showTimeline, setShowTimeline] = useState(false);
    const [expandedGraphs, setExpandedGraphs] = useState<Record<string, boolean>>({});
    const [sliderRange, setSliderRange] = useState<[number, number]>([0, 100]);

    const lastLog = logs[logs.length - 1];

    // Вычисляем видимые данные на основе диапазона слайдера
    const visibleData = useMemo(() => {
        const [start, end] = sliderRange;
        const slicedLogs = logs.slice(start, end); // Берем только нужный диапазон логов
        const data = slicedLogs.map((log) => {
            const values = collectValues(log.data.data);
            return {
                timestamp: log.timestamp,
                ...Object.fromEntries(values.map(({ key, value }) => [key, value])),
            };
        });
        console.log('Visible data computed:', data);
        return data;
    }, [logs, sliderRange]);

    // Логируем изменение visibleData
    useEffect(() => {
        console.log('Visible data updated:', visibleData);
    }, [visibleData]);

    // Автоматически обновляем слайдер при изменении данных
    useEffect(() => {
        const newEnd = logs.length;
        const newStart = Math.max(0, newEnd - 100); // Всегда последние 100 значений
        console.log('Updating slider range:', [newStart, newEnd]);
        setSliderRange([newStart, newEnd]);
    }, [lastLog]);

    // Логируем изменение sliderRange
    useEffect(() => {
        console.log('Slider range updated:', sliderRange);
    }, [sliderRange]);

    const toggleGraph = (key: string) => {
        console.log('Toggling graph for key:', key);
        setExpandedGraphs((prev) => ({
            ...prev,
            [key]: !prev[key],
        }));
    };

    return (
        <Dropdown key={topic} title={`Topic: ${topic}`}>
            <button onClick={() => setShowTimeline(!showTimeline)}>
                {showTimeline ? 'Hide Timeline' : 'Show Timeline'}
            </button>
            {showTimeline && <Timeline droneId={drone} topic={topic} />}
            <div>
                {Object.entries(lastLog.data.data).map(([key, value]) => (
                    <div key={key}>
                        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                            <button onClick={() => toggleGraph(key)}>
                                {expandedGraphs[key] ? 'Hide Graph' : 'Graph'}
                            </button>
                            <strong>{key}:</strong> {formatValue(value)}
                        </div>
                        {expandedGraphs[key] && (
                            <div style={{  marginBottom: '10px' }}>
                                <LineChartComponent
                                    data={visibleData}
                                    lines={collectValues(lastLog.data.data[key], key).map(
                                        ({ key: subKey }) => subKey,
                                    )}
                                    title={`${key} Over Time`}
                                />
                                <Slider
                                    min={0}
                                    max={logs.length}
                                    value={sliderRange}
                                    onChange={(value: any) => setSliderRange(value as [number, number])}
                                    range
                                    style={{marginTop: '45px'}}
                                />
                            </div>
                        )}
                    </div>
                ))}
            </div>
        </Dropdown>
    );
});