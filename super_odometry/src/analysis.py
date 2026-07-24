import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def load_and_analyze(csv_path, label):
    """Load CSV and return analysis results"""
    df = pd.read_csv(csv_path)
    
    # Calculate statistics
    stats = {
        'label': label,
        'runtime': df['timestamp'].max() - df['timestamp'].min(),
        'total_frames': df['total_keys_processed'].max(),
        'num_events': df['events'].max(),  # Updated: events instead of num_resets
        'avg_opt_time': df['opt_time_ms'].mean(),
        'max_opt_time': df['opt_time_ms'].max(),
        'avg_total_time': df['total_time_ms'].mean(),
        'max_total_time': df['total_time_ms'].max(),
        'avg_memory': df['memory_mb'].mean(),
        'peak_memory': df['memory_mb'].max(),
        'final_graph_size': df['graph_size'].iloc[-1] if len(df) > 0 else 0,
        'max_graph_size': df['graph_size'].max(),
        'total_marginalized': df['marginalized_keys'].max(),  # New: marginalized keys
        'avg_pose_history': df['pose_history_size'].mean(),  # New: pose history
        'max_pose_history': df['pose_history_size'].max(),
        'final_key': df['key'].max()  # New: final key value
    }
    
    # Identify event points (resets or marginalizations)
    df['event_occurred'] = df['events'].diff() > 0
    event_timestamps = df[df['event_occurred']]['timestamp'].values
    
    # Performance over time (compare early vs late performance)
    runtime = stats['runtime']
    if runtime > 20:  # If runtime > 20 seconds, compare first/last 10 seconds
        late_window = 10
        early_window = 10
    else:  # For shorter runs, use 25% of runtime
        late_window = runtime * 0.25
        early_window = runtime * 0.25
    
    late_perf = df[df['timestamp'] > df['timestamp'].max() - late_window]['opt_time_ms'].mean()
    early_perf = df[df['timestamp'] < df['timestamp'].min() + early_window]['opt_time_ms'].mean()
    stats['degradation'] = late_perf - early_perf
    
    # Calculate efficiency metrics
    if stats['total_frames'] > 0:
        stats['avg_frames_per_event'] = stats['total_frames'] / max(stats['num_events'], 1)
        stats['marginalization_ratio'] = stats['total_marginalized'] / stats['total_frames']
    else:
        stats['avg_frames_per_event'] = 0
        stats['marginalization_ratio'] = 0
    
    return df, stats, event_timestamps

def print_comparison(stats1, stats2):
    """Print comparison between two runs"""
    print("\n" + "="*80)
    print("PERFORMANCE COMPARISON")
    print("="*80)
    print(f"Dataset 1: {stats1['label']}")
    print(f"Dataset 2: {stats2['label']}")
    print("="*80)
    
    print(f"\n{'Metric':<35} {'Dataset 1':<15} {'Dataset 2':<15} {'Difference':<15}")
    print("-"*85)
    
    # Runtime
    print(f"{'Runtime (s)':<35} {stats1['runtime']:<15.2f} {stats2['runtime']:<15.2f} "
          f"{(stats2['runtime']-stats1['runtime']):<15.2f}")
    
    # Frames processed
    print(f"{'Total Frames Processed':<35} {stats1['total_frames']:<15} {stats2['total_frames']:<15} "
          f"{stats2['total_frames']-stats1['total_frames']:<15}")
    
    # Events (resets or marginalizations)
    print(f"{'Total Events':<35} {stats1['num_events']:<15} {stats2['num_events']:<15} "
          f"{stats2['num_events']-stats1['num_events']:<15}")
    
    # Keys marginalized
    print(f"{'Keys Marginalized':<35} {stats1['total_marginalized']:<15} {stats2['total_marginalized']:<15} "
          f"{stats2['total_marginalized']-stats1['total_marginalized']:<15}")
    
    # Graph size
    print(f"{'Final Graph Size':<35} {stats1['final_graph_size']:<15} {stats2['final_graph_size']:<15} "
          f"{stats2['final_graph_size']-stats1['final_graph_size']:<15}")
    
    print(f"{'Max Graph Size':<35} {stats1['max_graph_size']:<15} {stats2['max_graph_size']:<15} "
          f"{stats2['max_graph_size']-stats1['max_graph_size']:<15}")
    
    # Final key
    print(f"{'Final Key Value':<35} {stats1['final_key']:<15} {stats2['final_key']:<15} "
          f"{stats2['final_key']-stats1['final_key']:<15}")
    
    # Optimization time
    print(f"{'Avg Optimization (ms)':<35} {stats1['avg_opt_time']:<15.2f} {stats2['avg_opt_time']:<15.2f} "
          f"{(stats2['avg_opt_time']-stats1['avg_opt_time']):<15.2f}")
    
    print(f"{'Max Optimization (ms)':<35} {stats1['max_opt_time']:<15.2f} {stats2['max_opt_time']:<15.2f} "
          f"{(stats2['max_opt_time']-stats1['max_opt_time']):<15.2f}")
    
    # Total processing time
    print(f"{'Avg Total Time (ms)':<35} {stats1['avg_total_time']:<15.2f} {stats2['avg_total_time']:<15.2f} "
          f"{(stats2['avg_total_time']-stats1['avg_total_time']):<15.2f}")
    
    # Memory
    print(f"{'Avg Memory (MB)':<35} {stats1['avg_memory']:<15.2f} {stats2['avg_memory']:<15.2f} "
          f"{(stats2['avg_memory']-stats1['avg_memory']):<15.2f}")
    
    print(f"{'Peak Memory (MB)':<35} {stats1['peak_memory']:<15.2f} {stats2['peak_memory']:<15.2f} "
          f"{(stats2['peak_memory']-stats1['peak_memory']):<15.2f}")
    
    # Pose history
    print(f"{'Avg Pose History Size':<35} {stats1['avg_pose_history']:<15.2f} {stats2['avg_pose_history']:<15.2f} "
          f"{(stats2['avg_pose_history']-stats1['avg_pose_history']):<15.2f}")
    
    print(f"{'Max Pose History Size':<35} {stats1['max_pose_history']:<15.2f} {stats2['max_pose_history']:<15.2f} "
          f"{(stats2['max_pose_history']-stats1['max_pose_history']):<15.2f}")
    
    # Performance degradation
    print(f"{'Performance Degradation (ms)':<35} {stats1['degradation']:<15.2f} {stats2['degradation']:<15.2f} "
          f"{(stats2['degradation']-stats1['degradation']):<15.2f}")
    
    print("\n" + "="*80)
    print("EFFICIENCY ANALYSIS")
    print("="*80)
    
    # Graph size efficiency
    if stats2['final_graph_size'] > 0:
        graph_reduction = (1 - stats1['final_graph_size']/stats2['final_graph_size']) * 100
        print(f"Graph Size Reduction: {graph_reduction:.1f}%")
    
    # Optimization speedup
    if stats2['avg_opt_time'] > 0:
        opt_speedup = stats2['avg_opt_time'] / stats1['avg_opt_time']
        print(f"Optimization Speedup: {opt_speedup:.2f}x")
    
    # Memory efficiency
    if stats2['peak_memory'] > 0:
        memory_savings = stats2['peak_memory'] - stats1['peak_memory']
        memory_savings_pct = (memory_savings / stats2['peak_memory']) * 100
        print(f"Memory Savings: {memory_savings:.0f} MB ({memory_savings_pct:.1f}%)")
    
    # Marginalization efficiency
    print(f"Marginalization Ratio Dataset 1: {stats1['marginalization_ratio']:.2%}")
    print(f"Marginalization Ratio Dataset 2: {stats2['marginalization_ratio']:.2%}")
    
    # Events per frame
    print(f"Events per 100 frames Dataset 1: {(stats1['num_events']/max(stats1['total_frames'], 1)*100):.1f}")
    print(f"Events per 100 frames Dataset 2: {(stats2['num_events']/max(stats2['total_frames'], 1)*100):.1f}")

def plot_comparison(df1, df2, stats1, stats2, event_timestamps1, event_timestamps2):
    """Create comparison plots"""
    fig = plt.figure(figsize=(18, 12))
    
    # Create 2x2 subplot layout for focused comparison
    gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.3)
    
    # Convert to numpy arrays and normalize timestamps to start from 0
    t1 = df1['timestamp'].values - df1['timestamp'].min()
    t2 = df2['timestamp'].values - df2['timestamp'].min()
    event_timestamps1_norm = event_timestamps1 - df1['timestamp'].min()
    event_timestamps2_norm = event_timestamps2 - df2['timestamp'].min()
    
    label1 = stats1['label']
    label2 = stats2['label']
    
    # 1. Graph Size Comparison
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t1, df1['graph_size'].values, 'b-', label=f'{label1}', linewidth=2)
    ax1.plot(t2, df2['graph_size'].values, 'r--', label=f'{label2}', linewidth=2)
    
    # Mark events
    for event_time in event_timestamps1_norm:
        ax1.axvline(x=event_time, color='b', linestyle=':', alpha=0.6, linewidth=1)
    for event_time in event_timestamps2_norm:
        ax1.axvline(x=event_time, color='r', linestyle=':', alpha=0.6, linewidth=1)
    
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Graph Size (nodes)', fontsize=12)
    ax1.set_title('Graph Size Comparison Over Time', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=11)
    ax1.grid(True, alpha=0.3)
    
    # 2. Optimization Time Comparison
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t1, df1['opt_time_ms'].values, 'b-', label=f'{label1}', linewidth=2)
    ax2.plot(t2, df2['opt_time_ms'].values, 'r--', label=f'{label2}', linewidth=2)
    ax2.axhline(y=50, color='orange', linestyle='--', label='50ms threshold', linewidth=1.5)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Optimization Time (ms)', fontsize=12)
    ax2.set_title('Optimization Performance', fontsize=13, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    # 3. Memory Usage Comparison
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(t1, df1['memory_mb'].values, 'b-', label=f'{label1}', linewidth=2)
    ax3.plot(t2, df2['memory_mb'].values, 'r--', label=f'{label2}', linewidth=2)
    ax3.set_xlabel('Time (s)', fontsize=12)
    ax3.set_ylabel('Memory (MB)', fontsize=12)
    ax3.set_title('Memory Usage', fontsize=13, fontweight='bold')
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3)
    
    plt.suptitle(f'IMU Preintegration Performance: {label1} vs {label2}', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    
    # Create efficiency analysis as a separate figure
    create_efficiency_plot(stats1, stats2)
    
    return fig

def create_efficiency_plot(stats1, stats2):
    """Create efficiency analysis bar chart"""
    fig, ax = plt.subplots(figsize=(12, 8))
    
    label1 = stats1['label']
    label2 = stats2['label']
    
    # Calculate efficiency improvements (negative means dataset1 is better)
    improvements = []
    improvement_labels = []
    
    # Graph Size Reduction
    if stats2['final_graph_size'] > 0:
        graph_improvement = (1 - stats1['final_graph_size']/stats2['final_graph_size']) * 100
        improvements.append(graph_improvement)
        improvement_labels.append('Graph Size\nReduction')
    
    # Optimization Time Improvement
    if stats1['avg_opt_time'] > 0:
        opt_improvement = (1 - stats1['avg_opt_time']/stats2['avg_opt_time']) * 100
        improvements.append(opt_improvement)
        improvement_labels.append('Optimization\nSpeed Gain')
    
    # Memory Efficiency
    if stats2['peak_memory'] > 0:
        memory_improvement = (1 - stats1['peak_memory']/stats2['peak_memory']) * 100
        improvements.append(memory_improvement)
        improvement_labels.append('Memory\nReduction')
    
    # Runtime Efficiency
    if stats2['runtime'] > 0:
        runtime_improvement = (1 - stats1['runtime']/stats2['runtime']) * 100
        improvements.append(runtime_improvement)
        improvement_labels.append('Runtime\nEfficiency')
    
    # Max Optimization Time Improvement
    if stats2['max_opt_time'] > 0:
        max_opt_improvement = (1 - stats1['max_opt_time']/stats2['max_opt_time']) * 100
        improvements.append(max_opt_improvement)
        improvement_labels.append('Peak Opt Time\nReduction')
    
    # Color bars based on improvement (green = positive, red = negative)
    colors = ['green' if imp > 0 else 'red' for imp in improvements]
    
    # Create bar chart
    bars = ax.bar(improvement_labels, improvements, color=colors, alpha=0.7, 
                  edgecolor='black', linewidth=1.2)
    
    # Customize the plot
    ax.set_ylabel('Improvement (%)', fontsize=14, fontweight='bold')
    ax.set_title(f'Efficiency Analysis: {label1} vs {label2}\n(Positive = {label1} Better, Negative = {label2} Better)', 
                 fontsize=16, fontweight='bold', pad=20)
    ax.axhline(y=0, color='black', linestyle='-', linewidth=1)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, improvement in zip(bars, improvements):
        height = bar.get_height()
        label_y = height + (2 if height >= 0 else -5)
        va = 'bottom' if height >= 0 else 'top'
        
        ax.text(bar.get_x() + bar.get_width()/2., label_y,
                f'{improvement:.1f}%', ha='center', va=va, 
                fontsize=12, fontweight='bold')
    

    
    # Rotate x-axis labels for better readability
    plt.xticks(rotation=45, ha='right')
    
    plt.tight_layout()
    plt.savefig('efficiency_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    return fig

def main():

    csv_file1 = '/home/avnish/Desktop/final_report/no_smart_no_fixed/imu_performance_traditional_without_reset_20250807_005336.csv'
    csv_file2 = '/home/avnish/Desktop/final_report/test_fixed/imu_performance_fixed_lag_incremental_20250807_110334.csv'
    
    
    # Check if files exist
    if not os.path.exists(csv_file1):
        print(f"Error: File {csv_file1} not found!")
        print(f"Please update the path in the main() function")
        return
    if not os.path.exists(csv_file2):
        print(f"Error: File {csv_file2} not found!")
        print(f"Please update the path in the main() function")
        return
    
    print(f"\nAnalyzing performance comparison:")
    print(f"  File 1: {os.path.basename(csv_file1)}")
    print(f"  File 2: {os.path.basename(csv_file2)}")
    
    # Load and analyze both files
    df1, stats1, events1 = load_and_analyze(csv_file1, "Traditional ISAM2")
    df2, stats2, events2 = load_and_analyze(csv_file2, "Fixed-Lag Smoother")
    
    # Print comparison
    print_comparison(stats1, stats2)
    
    # Create comparison plots
    fig = plot_comparison(df1, df2, stats1, stats2, events1, events2)
    
    # Generate output filename
    output_file = 'imu_performance_comparison_modified.png'
    
    # Save the main figure
    fig.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nComparison plot saved to: {output_file}")
    print(f"Efficiency analysis saved to: efficiency_analysis.png")
    
    plt.show()

if __name__ == "__main__":
    main()