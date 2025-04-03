import React from 'react';
import ReactECharts from 'echarts-for-react';

interface EChartsComponentProps {
    data: Array<{ x: string[]; y: number[]; name: string }>;
    title?: string;
}

export const EChartsComponent: React.FC<EChartsComponentProps> = ({ data, title }) => {
    const option = {
        title: {
            text: title,
            left: 'center',
        },
        tooltip: {
            trigger: 'axis',
        },
        xAxis: {
            type: 'category',
            data: data[0]?.x || [],
            name: 'Time',
            boundaryGap: false,
        },
        yAxis: {
            type: 'value',
            name: 'Value',
        },
        series: data.map(({ y, name }) => ({
            name,
            type: 'line',
            data: y,
            smooth: true,
        })),
        legend: {
            data: data.map(({ name }) => name),
            bottom: 0,
        },
        dataZoom: [
            {
                type: 'inside',
                start: 0,
                end: 10,
            },
            {
                type: 'slider',
                start: 0,
                end: 10,
            },
        ],
    };

    return <ReactECharts option={option} style={{ height: '400px', width: '100%' }} />;
};