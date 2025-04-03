import React, { memo } from 'react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer } from 'recharts';

interface DataPoint {
    timestamp: string;
    [key: string]: any;
}

interface LineChartComponentProps {
    data: DataPoint[];
    lines: string[];
    title?: string;
}

export const LineChartComponent: React.FC<LineChartComponentProps> = memo(({ data, lines, title }) => {
    const colors = ['#8884d8', '#82ca9d', '#ffc658', '#ff7300', '#413ea0'];

    return (
        <div style={{ width: '100%', height: 300 }}>
            {title && <h4>{title}</h4>}
            <ResponsiveContainer>
                <LineChart data={data}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis dataKey="timestamp" />
                    <YAxis />
                    <Tooltip />
                    <Legend />
                    {lines.map((line, index) => (
                        <Line
                            key={line}
                            type="monotone"
                            dataKey={line}
                            stroke={colors[index % colors.length]}
                            activeDot={{ r: 8 }}
                        />
                    ))}
                </LineChart>
            </ResponsiveContainer>
        </div>
    );
});