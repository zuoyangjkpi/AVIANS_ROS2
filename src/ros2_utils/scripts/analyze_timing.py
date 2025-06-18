#!/usr/bin/env python3
"""
Timing Analysis Script - Analyze CSV logs from timestamp monitor
Usage: python3 analyze_timing.py /tmp/drone_timing_analysis.csv
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np
from pathlib import Path

def analyze_timing_data(csv_file):
    """Analyze timing data from CSV log file"""
    
    if not Path(csv_file).exists():
        print(f"Error: File {csv_file} not found!")
        return
    
    # Read CSV data
    print(f"Loading timing data from {csv_file}...")
    df = pd.read_csv(csv_file)
    
    if df.empty:
        print("Error: CSV file is empty!")
        return
    
    print(f"Loaded {len(df)} timestamp records")
    print(f"Topics monitored: {df['topic'].nunique()}")
    print(f"Time range: {df['timestamp'].min():.3f} to {df['timestamp'].max():.3f} seconds")
    
    # 1. Frequency Analysis
    print("\n========== FREQUENCY ANALYSIS ==========")
    freq_stats = df.groupby('topic')['frequency_hz'].agg(['mean', 'std', 'count']).round(2)
    freq_stats.columns = ['Avg_Hz', 'Std_Hz', 'Message_Count']
    freq_stats = freq_stats.sort_values('Avg_Hz', ascending=False)
    print(freq_stats)
    
    # 2. Synchronization Analysis
    print("\n========== SYNCHRONIZATION ANALYSIS ==========")
    sync_stats = df.groupby('topic')['sync_offset_ms'].agg(['mean', 'std', 'min', 'max']).round(2)
    sync_stats.columns = ['Mean_Offset_ms', 'Std_Offset_ms', 'Min_Offset_ms', 'Max_Offset_ms']
    sync_stats['Sync_Quality'] = sync_stats['Std_Offset_ms'].apply(
        lambda x: 'Excellent' if x < 10 else 'Good' if x < 50 else 'Poor' if x < 100 else 'Very Poor'
    )
    sync_stats = sync_stats.sort_values('Std_Offset_ms')
    print(sync_stats)
    
    # 3. Drift Analysis
    print("\n========== TIMESTAMP DRIFT ANALYSIS ==========")
    drift_stats = df.groupby('topic')['drift_ms'].agg(['mean', 'std', 'min', 'max']).round(2)
    drift_stats.columns = ['Mean_Drift_ms', 'Std_Drift_ms', 'Min_Drift_ms', 'Max_Drift_ms']
    drift_stats = drift_stats.sort_values('Mean_Drift_ms', key=abs, ascending=False)
    print(drift_stats)
    
    # 4. Critical Issues Detection
    print("\n========== CRITICAL TIMING ISSUES ==========")
    
    # Find topics with large sync offsets
    large_offsets = df[abs(df['sync_offset_ms']) > 100]
    if not large_offsets.empty:
        print("Topics with large sync offsets (>100ms):")
        problem_topics = large_offsets.groupby('topic')['sync_offset_ms'].agg(['count', 'mean']).round(2)
        problem_topics.columns = ['Occurrences', 'Avg_Offset_ms']
        print(problem_topics.sort_values('Avg_Offset_ms', key=abs, ascending=False))
    else:
        print("✓ No critical sync offset issues found")
    
    # Find topics with unstable frequencies
    unstable_freq = freq_stats[freq_stats['Std_Hz'] > freq_stats['Avg_Hz'] * 0.2]
    if not unstable_freq.empty:
        print("\nTopics with unstable frequencies (>20% variation):")
        print(unstable_freq[['Avg_Hz', 'Std_Hz']])
    else:
        print("✓ No unstable frequency issues found")
    
    # Find topics with large timestamp drift
    large_drift = drift_stats[abs(drift_stats['Mean_Drift_ms']) > 50]
    if not large_drift.empty:
        print("\nTopics with large timestamp drift (>50ms):")
        print(large_drift[['Mean_Drift_ms', 'Std_Drift_ms']])
    else:
        print("✓ No significant timestamp drift issues found")
    
    # 5. Generate Plots
    print("\n========== GENERATING PLOTS ==========")
    
    # Get unique topics for plotting
    topics = df['topic'].unique()
    n_topics = len(topics)
    
    if n_topics > 0:
        # Create subplot grid
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Drone System Timing Analysis', fontsize=16)
        
        # Plot 1: Sync offsets over time
        ax1 = axes[0, 0]
        for topic in topics[:8]:  # Limit to first 8 topics for readability
            topic_data = df[df['topic'] == topic]
            if len(topic_data) > 1:
                ax1.plot(topic_data['timestamp'], topic_data['sync_offset_ms'], 
                        label=topic.split('/')[-1], alpha=0.7)
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('Sync Offset (ms)')
        ax1.set_title('Synchronization Offsets Over Time')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='black', linestyle='--', alpha=0.5)
        
        # Plot 2: Frequency distribution
        ax2 = axes[0, 1]
        freq_data = freq_stats['Avg_Hz'].dropna()
        if len(freq_data) > 0:
            bars = ax2.bar(range(len(freq_data)), freq_data.values)
            ax2.set_xlabel('Topics')
            ax2.set_ylabel('Frequency (Hz)')
            ax2.set_title('Average Frequencies by Topic')
            ax2.set_xticks(range(len(freq_data)))
            ax2.set_xticklabels([t.split('/')[-1] for t in freq_data.index], rotation=45, ha='right')
            ax2.grid(True, alpha=0.3)
            
            # Color bars based on frequency
            for i, bar in enumerate(bars):
                if freq_data.iloc[i] < 1:
                    bar.set_color('red')
                elif freq_data.iloc[i] < 10:
                    bar.set_color('orange')
                else:
                    bar.set_color('green')
        
        # Plot 3: Drift distribution
        ax3 = axes[1, 0]
        drift_data = drift_stats['Mean_Drift_ms']
        if len(drift_data) > 0:
            colors = ['red' if abs(x) > 50 else 'orange' if abs(x) > 20 else 'green' for x in drift_data.values]
            bars = ax3.bar(range(len(drift_data)), drift_data.values, color=colors)
            ax3.set_xlabel('Topics')
            ax3.set_ylabel('Mean Drift (ms)')
            ax3.set_title('Timestamp Drift by Topic')
            ax3.set_xticks(range(len(drift_data)))
            ax3.set_xticklabels([t.split('/')[-1] for t in drift_data.index], rotation=45, ha='right')
            ax3.grid(True, alpha=0.3)
            ax3.axhline(y=0, color='black', linestyle='--', alpha=0.5)
        
        # Plot 4: Sync offset histogram
        ax4 = axes[1, 1]
        all_offsets = df['sync_offset_ms'].dropna()
        if len(all_offsets) > 0:
            ax4.hist(all_offsets, bins=50, alpha=0.7, edgecolor='black')
            ax4.set_xlabel('Sync Offset (ms)')
            ax4.set_ylabel('Frequency')
            ax4.set_title('Distribution of Sync Offsets')
            ax4.grid(True, alpha=0.3)
            ax4.axvline(x=0, color='red', linestyle='--', label='Perfect Sync')
            ax4.axvline(x=all_offsets.mean(), color='orange', linestyle='--', label=f'Mean: {all_offsets.mean():.1f}ms')
            ax4.legend()
        
        plt.tight_layout()
        
        # Save plots
        plot_file = csv_file.replace('.csv', '_analysis.png')
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"Plots saved to: {plot_file}")
        
        # Show plots
        plt.show()
    
    # 6. Generate Summary Report
    print("\n========== SUMMARY REPORT ==========")
    
    # Overall system health
    total_messages = len(df)
    sync_issues = len(df[abs(df['sync_offset_ms']) > 100])
    drift_issues = len(df[abs(df['drift_ms']) > 50])
    
    sync_health = (1 - sync_issues / total_messages) * 100 if total_messages > 0 else 0
    drift_health = (1 - drift_issues / total_messages) * 100 if total_messages > 0 else 0
    
    print(f"Total messages analyzed: {total_messages}")
    print(f"Synchronization health: {sync_health:.1f}% ({sync_issues} issues)")
    print(f"Timestamp drift health: {drift_health:.1f}% ({drift_issues} issues)")
    
    # Recommendations
    print("\n--- RECOMMENDATIONS ---")
    
    if sync_health < 90:
        print("⚠️  CRITICAL: Poor synchronization detected!")
        print("   - Check clock synchronization across nodes")
        print("   - Verify use_sim_time parameter consistency")
        print("   - Consider increasing message buffer sizes")
    elif sync_health < 95:
        print("⚠️  WARNING: Some synchronization issues detected")
        print("   - Monitor timing tolerance parameters")
        print("   - Check network latency in distributed setup")
    else:
        print("✅ Synchronization health is good")
    
    if drift_health < 90:
        print("⚠️  CRITICAL: Large timestamp drift detected!")
        print("   - Check system clock stability") 
        print("   - Verify Gazebo simulation real-time factor")
        print("   - Monitor CPU load and processing delays")
    elif drift_health < 95:
        print("⚠️  WARNING: Some timestamp drift detected")
        print("   - Monitor system performance")
        print("   - Consider optimizing node processing times")
    else:
        print("✅ Timestamp drift is within acceptable limits")
    
    # Top problematic topics
    if not large_offsets.empty:
        print("\n--- TOP PROBLEMATIC TOPICS ---")
        worst_topics = large_offsets.groupby('topic').size().sort_values(ascending=False).head(3)
        for i, (topic, count) in enumerate(worst_topics.items(), 1):
            print(f"  {i}. {topic}: {count} timing violations")
    
    print("\n==========================================")
    
    return {
        'sync_health': sync_health,
        'drift_health': drift_health,
        'total_messages': total_messages,
        'sync_issues': sync_issues,
        'drift_issues': drift_issues
    }

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_timing.py <csv_file>")
        print("Example: python3 analyze_timing.py /tmp/drone_timing_analysis.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    results = analyze_timing_data(csv_file)
    
    if results:
        # Exit with error code if critical issues found
        if results['sync_health'] < 80 or results['drift_health'] < 80:
            print("\n❌ CRITICAL timing issues detected!")
            sys.exit(1)
        elif results['sync_health'] < 90 or results['drift_health'] < 90:
            print("\n⚠️  Some timing issues detected")
            sys.exit(2)
        else:
            print("\n✅ Timing analysis completed successfully")
            sys.exit(0)

if __name__ == "__main__":
    main()