#!/usr/bin/env bash

read -r header

cat <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<OpenSimDocument Version="40000">
	<DistanceWeightSet name="all_uniform_dweights">
		<objects>
EOF

IFS=$'\t' read -ra cols <<< "$header"

for name in "${cols[@]}"; do
    [[ "$name" == "time" ]] && continue

    cat <<EOF
			<DistanceWeight name="$name">
				<weight>1</weight>
			</DistanceWeight>
EOF
done

cat <<EOF
		</objects>
	</DistanceWeightSet>
</OpenSimDocument>
EOF