#!/usr/bin/env bash

input="data/bsm_Scale_MarkerSet.xml"
output="data/bsm_IK_Tasks_uniform.xml"

echo '<?xml version="1.0" encoding="UTF-8"?>' > "$output"
echo '<IKTaskSet name="bsm_base_IK">' >> "$output"
echo '    <objects>' >> "$output"

grep '<Marker name=' "$input" | sed -E 's/.*name="([^"]+)".*/\1/' | while read name
do
cat >> "$output" <<EOF
        <IKMarkerTask name="$name">
            <apply> true </apply>
            <weight> 1.00000000 </weight>
        </IKMarkerTask>
EOF
done

echo '    </objects>' >> "$output"
echo '</IKTaskSet>' >> "$output"

echo "Written to $output"