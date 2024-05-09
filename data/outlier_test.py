import pandas as pd

def replace_outliers_within_group(df, group_columns, target_column):
    # Function to calculate and replace outliers within each group
    def replace(group):
        Q1 = group[target_column].quantile(0.25)
        Q3 = group[target_column].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        mean_value = group[(group[target_column] >= lower_bound) & (group[target_column] <= upper_bound)][target_column].mean()
        group[target_column] = group[target_column].apply(lambda x: mean_value if x < lower_bound or x > upper_bound else x)
        return group

    # Apply the function to each subgroup
    return df.groupby(group_columns).apply(replace)

# Example usage:
# Assuming df is your DataFrame and it has columns 'autonomy', 'ring', and 'time'
# Replace outliers in the 'time' column within each 'autonomy' and 'ring' group
df = pd.DataFrame({
    'autonomy': ['low', 'high', 'low', 'high', 'low', 'high', 'low', 'high'],
    'ring': ['on', 'on', 'off', 'off', 'on', 'on', 'off', 'off'],
    'time': [120, 150, 130, 160, 112, 148, 180, 110]
})

cleaned_df = replace_outliers_within_group(df, ['autonomy', 'ring'], 'time')
print(cleaned_df)
