.PHONY: demo down logs shell bag-info download-bag clean-saved

demo:
	docker compose up --build

down:
	docker compose down

logs:
	docker compose logs -f

shell:
	docker compose run --rm ros2-web-lab shell

bag-info:
	docker compose run --rm ros2-web-lab bag-info

download-bag:
	docker compose run --rm ros2-web-lab download-bag

clean-saved:
	rm -rf saved/raw saved/metadata saved/undistorted
	mkdir -p saved
